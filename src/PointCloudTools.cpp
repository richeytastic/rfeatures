#include "PointCloudTools.h"
using RFeatures::PointCloudTools;

#include <cassert>
#include <iostream>
using std::cout;
using std::cerr;
using std::endl;
#include <cstdlib>
#include <vector>
using std::vector;
#include <fstream>
#include <sstream>
#include <exception>



bool isOriginPoint( double x, double y, double z)
{
    static const double MIN_ZERO = 0.00000001;
    return fabs(x) < MIN_ZERO && fabs(y) < MIN_ZERO && fabs(z) < MIN_ZERO;
}   // end isOriginPoint



void tokenize( const string& ln, vector<string>& toks, char delim=' ')
{
    toks.clear();

    const int lnSz = ln.size();
    string curTok = "";
    for ( int i = 0; i < lnSz; ++i)
    {
        const char c = ln[i];
        if ( c == delim && !curTok.empty())
        {
            toks.push_back(curTok);
            curTok = "";
        }   // end if
        else if ( c != delim)
            curTok += c;
    }   // end for

    if ( !curTok.empty())
        toks.push_back(curTok);
}   // end tokenize



// Open the points file and find out what kind of data it contains.
// On return rows and cols contains either the number of rows and columns in
// the organised data, or if cols == 0 on return, rows will contain the number
// of points or will be 0 if no header line was given.
void loadPointsFile( const string& fname, std::ifstream& ifs, bool& hasIntensity, bool& hasColour, int& rows, int& cols)
{
    rows = 0;
    cols = 0;
    hasIntensity = false;
    hasColour = false;

    try
    {
        ifs.open( fname.c_str());
        if ( ifs.is_open())
        {
            int afterHeaderPos = 0;

            // If this is a PTS file, it might contain a single line at the top stating the number
            // of points in the file. Or it may contain two numbers giving the number of rows and
            // columns (respectively) in the file for organised point data.
            // We want to remove this line if it exists.
            string ln;
            std::getline( ifs,ln);
            vector<string> toks;
            tokenize( ln, toks, ' ');
            const bool hasHeader = toks.size() < 3; // Does this file have a header?

            if ( hasHeader)
            {
                //cerr << "Header tokens: " << toks.size() << endl;
                afterHeaderPos = ifs.tellg();
                // Get the header info
                std::istringstream iss(ln);
                iss >> rows;
                if ( toks.size() > 1)
                    iss >> cols;

                //read in the next line and get tokens to find number of fields
                std::getline( ifs,ln);
                tokenize( ln, toks, ' ');
            }   // end if

            //cerr << "AHP: " << afterHeaderPos << endl;
            //cerr << "Field tokens: " << toks.size() << endl;
            const int numToks = toks.size();
            hasColour = numToks >= 6;
            hasIntensity = numToks == 4 || numToks == 7 || numToks == 9;
            ifs.seekg( afterHeaderPos); // Set read stream back to after header
        }   // end if
    }   // end try
    catch ( const std::exception& e)
    {
        cerr << "Unable to read point file " << fname << endl;
        cerr << e.what() << endl;
    }   // end catch
}   // end loadPointsFile



PointCloud::Ptr PointCloudTools::loadText( const string& fname)
{
    PointCloud::Ptr pc;

    int rows = 0, cols = 0;
    bool hasIntensity = false, hasColour = false;
    std::ifstream ifs;
    loadPointsFile( fname, ifs, hasIntensity, hasColour, rows, cols);
    if ( !ifs.is_open())
        return pc;

    int zeroPoints = 0;
    if ( cols == 0)
        pc = PointCloud::create();  // Unorganised
    else
        pc = PointCloud::create( cols, rows);   // Organised

    string ln;
    double x, y, z;
    while ( std::getline( ifs, ln))
    {
        if ( ln.empty())
            continue;

        std::istringstream iss(ln);
        int rowIdx = 0, colIdx = 0;
        int r = 255, g = 255, b = 255;  // Default point colour is white
        double intensity = 1;

        if ( cols > 0)   // Only for organised point clouds
            iss >> rowIdx >> colIdx;

        iss >> x >> y >> z;
        if ( hasIntensity)  // Intensity is not used
            iss >> intensity;
        if ( hasColour)
            iss >> r >> g >> b;

        if ( cols > 0)  // Organised point cloud
            pc->set( rowIdx, colIdx, x, y, z, byte(r), byte(g), byte(b));
        else    // Unorganised point cloud
        {
            // Only add points that aren't at the origin (0,0,0)
            if ( !isOriginPoint(x,y,z))
                pc->add( x, y, z, byte(r), byte(g), byte(b));
            else
                zeroPoints++;
        }   // end else
    }   // end while
    ifs.close();

    // Warn if number of points read in does not match the number of points stated
    // in the point file's header line (whether organised or not).
    if ( rows > 0)
    {
        int statedNumPoints = rows - zeroPoints;
        if ( cols > 0)
            statedNumPoints = rows * cols;

        if ( pc->size() != statedNumPoints)
            cerr << "WARNING: Number of points in file does not match file's declared number!" << endl;
    }   // end if

    switch ( pc->size())
    {
        case 0:
            cerr << "File didn't contain any valid points!" << endl;
            break;
        case 1:
            cerr << "File must contain more than a single point!" << endl;
            break;
    }   // end switch

    if ( pc->size() <= 1)
        pc.reset();

    if ( zeroPoints > 0)
    {
        cout << zeroPoints << " zero point" << ( zeroPoints > 1 ? "s " : " ")
             << "(points at origin) not added to unorganised point cloud." << endl;
    }   // end if

    return pc;
}   // end loadText



PointCloud::Ptr PointCloudTools::interpolate( const PointCloud::Ptr pc)
{
    assert( pc->isOrganised());
    const int rows = pc->rows();
    const int cols = pc->cols();
    const int halfRows = rows/2;

    PointCloud::Ptr npc = PointCloud::create(cols,rows);

    //cv::Mat cimg = cv::Mat::zeros( rows, cols, CV_8UC3);
    cv::Mat oimg = cv::Mat::zeros( rows, cols, CV_8UC1);
    cv::Mat timg = cv::Mat::zeros( rows, cols, CV_8UC1);
    byte r, g, b;
    float x, y, z;

    double maxD = -DBL_MAX;
    double minD = DBL_MAX;
    for ( int i = 0; i < rows; ++i)
    {
        float depthDiffTotal = 0;
        float lastDepth = 0;
        const PointXYZRGB* lastPt;
        for ( int j = 0; j < cols; ++j)
        {
            const PointXYZRGB* pt = &pc->at(i,j);
            PointCloud::unpackPoint( *pt, x, y, z, r, g, b);

            const float depth = z;
            if ( depth > maxD)
                maxD = depth;
            if ( depth > 0 && depth < minD)
                minD = depth;

            // Check for a change in depth missing values in subsequent columns
            if ( lastDepth == 0 && depth > 0)
                depthDiffTotal += 1./depth;
            else if ( lastDepth > 0 && depth == 0)
                depthDiffTotal += 1./lastDepth;
            lastDepth = depth;

            if ( depth > 0)
            {
                timg.at<unsigned char>(rows-i-1, cols-j-1) = 255;
                oimg.at<unsigned char>(rows-i-1, cols-j-1) = 255;
                //cimg.at<cv::Vec3b>(rows-i-1, cols-j-1) = cv::Vec3b(b, g, r);
                npc->at(i,j) = *pt;
                lastPt = pt;
            }   // end if
        }   // end for

        if ( i > halfRows && depthDiffTotal > 6)
        {
            for ( int j = cols-1; j >= 0; --j)
            {
                PointXYZRGB* pt = &npc->at(i,j);
                PointCloud::unpackPoint( *pt, x, y, z, r, g, b);
                if ( z == 0)
                {
                    *pt = *lastPt;
                    PointCloud::unpackPoint( *pt, x, y, z, r, g, b);
                }   // end if

                if ( z > 0)
                    lastPt = pt;

                timg.at<unsigned char>(rows-i-1, cols-j-1) = 255;
                //cimg.at<cv::Vec3b>(rows-i-1, cols-j-1) = cv::Vec3b(b, g, r);
            }   // end for
        }   // end if
    }   // end for

    //cout << "Min, max depth = " << minD << ", " << maxD << endl;

    // Morphological closure on the depth threshold image
    cv::Mat elem22( 2, 2, CV_8U, cv::Scalar(1));
    cv::Mat c22;
    cv::morphologyEx( timg, c22, cv::MORPH_CLOSE, elem22);

    //RFeatures::showImage( oimg, "Original Depth", true);

    for ( int i = 0; i < rows; ++i)
    {
        const PointXYZRGB* intPt = 0;  // Point to interpolate with

        for ( int j = 0; j < cols; ++j)
        {
            PointXYZRGB& pt = npc->at(i,j);

            // Do we need to find a valid adjacent point to interpolate with?
            if (( pt.z == 0) && ( c22.at<unsigned char>(rows-i-1, cols-j-1) > 0))
            {
                if ( intPt == 0)
                {
                    // Try to find a valid point to interpolate with from a column to the right
                    int k = j;
                    while ( k < cols - 1 && (intPt == 0 || intPt->z == 0))
                        intPt = &npc->at(i,++k);
                }   // end if

                pt = *intPt;
            }   // end if

            PointCloud::unpackPoint( pt, x, y, z, r, g, b);
            if ( z > 0)
                intPt = &pt;
           
            //cimg.at<cv::Vec3b>(rows-i-1, cols-j-1) = cv::Vec3b(b, g, r);
            //timg.at<unsigned char>(rows-i-1, cols-j-1) = z > 0 ? int(255.0 - z) : 0;
        }   // end for
    }   // end for

    return npc;
}   // end interpolatePointCloud



vector<PointCloud::Ptr> PointCloudTools::createFaces( const cv::Vec3d& frontVec, const cv::Vec3d& upVec,
                        const cv::Vec3d& camPos, const PointCloud::Ptr pc, const cv::Vec3d scale)
{
    const cv::Vec3d nscale( 1./scale[0], 1./scale[1], 1./scale[2]);

    // Construct initial vectors correcting for vehicle orientation front face
    cv::Vec3d vecs[4];  // Focal direction vectors indexed from 0 to 3 as front, left, rear, right respectively
    vecs[0] = frontVec;
    vecs[1] = upVec.cross(frontVec); // Left
    vecs[2] = -frontVec; // Rear
    vecs[3] = -vecs[1]; // Right

    cv::Matx33d coordFrames[4];  // Front, left, rear and right point cloud coordinate frames

    // Front
    coordFrames[0] = cv::Matx33d( vecs[3][0], vecs[3][1], vecs[3][2],   // X - right view
                                  upVec[0], upVec[1], upVec[2],         // Y - up
                                  vecs[0][0], vecs[0][1], vecs[0][2]);  // Z - front
    // Left
    coordFrames[1] = cv::Matx33d( vecs[0][0], vecs[0][1], vecs[0][2],   // X - front view
                                  upVec[0], upVec[1], upVec[2],         // Y - up
                                  vecs[1][0], vecs[1][1], vecs[1][2]);  // Z - left
    // Rear
    coordFrames[2] = cv::Matx33d( vecs[1][0], vecs[1][1], vecs[1][2],   // X - left view
                                  upVec[0], upVec[1], upVec[2],         // Y - up
                                  vecs[2][0], vecs[2][1], vecs[2][2]);  // Z - rear
    // Right
    coordFrames[3] = cv::Matx33d( vecs[2][0], vecs[2][1], vecs[2][2],   // X - rear view
                                  upVec[0], upVec[1], upVec[2],         // Y - up
                                  vecs[3][0], vecs[3][1], vecs[3][2]);  // Z - right

    const cv::Vec3d downVec = -upVec;

    const int rows = 512;
    const int cols = 512;
    const double focLen = 256;

    vector<PointCloud::Ptr> pclouds(4);
    int faceCount[4] = {0, 0, 0, 0};
    int totCount = 0;

    pclouds[0] = PointCloud::create(cols,rows);  // Front
    pclouds[1] = PointCloud::create(cols,rows);  // Left
    pclouds[2] = PointCloud::create(cols,rows);  // Rear
    pclouds[3] = PointCloud::create(cols,rows);  // Right

    byte r, g, b;
    cv::Vec3d gpos;
    const int numPts = pc->size();
    for ( int i = 0; i < numPts; ++i)
    {
        pc->from( i, gpos[0], gpos[1], gpos[2], r, g, b);
        gpos -= camPos; // Set with respect to the camera's position
    
        // Calc projections of point in coordinate frames for the four different
        // view directions and select the one with the largest depth (that isn't up or down)
        double maxvd = std::max( fabs(upVec.dot(gpos)), fabs(downVec.dot(gpos)));
        int bestv = -1;
        for ( int j = 0; j < 4; ++j)
        {
            const double vd = vecs[j].dot(gpos);
            if ( vd > maxvd)
            {
                maxvd = vd;
                bestv = j;
            }   // end if
        }   // end for

        if ( bestv == -1)   // Point not in a valid horizontal view (front, left, rear or right) so skip
            continue;

        // Project the point into the local coordinate frame for the chosen view
        const cv::Vec3d vVec = (coordFrames[bestv] * gpos).mul(nscale);

        // The correct point cloud image x and y are found as similar triangles of the
        // point's perspective projection in the local coordinate frame of this view.
        const int x = int(focLen/vVec[2] * vVec[0]) + focLen;
        const int y = int(focLen/vVec[2] * -vVec[1]) + focLen;

        // Don't set if the existing point is closer (but not zero - i.e. unset)
        const PointXYZRGB& pt = pclouds[bestv]->at( y, x);
        if ( pt.z > 0 && pt.z < vVec[2])
            continue;

        pclouds[bestv]->set( y, x, -vVec[0], vVec[1], vVec[2], r, g, b);
        faceCount[bestv]++;
        totCount++;
    }   // end for

    // Check for low point count on any faces
    bool pass = true;
    for (int i = 0; i < 4; ++i)
    {
        if ( float(faceCount[i])/totCount < 0.05)
        {
            pass = false;
            break;
        }   // end if
    }   // end for

    if ( !pass)
        return vector<PointCloud::Ptr>();   // Empty

    return pclouds;
}   // end createFaces



View::Ptr PointCloudTools::createView( const PointCloud::Ptr pc,
        const cv::Vec3d& pos, const cv::Vec3d& fv, const cv::Vec3d& upv)
{
    assert( pc->isOrganised());
    const int rows = pc->rows();
    const int cols = pc->cols();

    View::Ptr view = View::create( cols, rows);
    view->posVec = pos;
    view->focalVec = fv;
    view->upVec = upv;

    byte r, g, b;
    double x, y, z;
    for ( int i = 0; i < rows; ++i)
    {
        cv::Vec3b* viewColRow = view->img2d.ptr<cv::Vec3b>(i);
        cv::Vec3f* viewPtsRow = view->points.ptr<cv::Vec3f>(i);
        float* viewRngRow = view->rngImg.ptr<float>(i);

        for ( int j = 0; j < cols; ++j)
        {
            cv::Vec3b& bgr = viewColRow[j];
            cv::Vec3f& pts = viewPtsRow[j];
            const PointXYZRGB& pt = pc->at(i,j);
            PointCloud::unpackPoint( pt, pts[0], pts[1], pts[2], bgr[2], bgr[1], bgr[0]);
            viewRngRow[j] = pts[2];
        }   // end for
    }   // end for

    return view;
}   // end createView


