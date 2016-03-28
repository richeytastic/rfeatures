#include "ViewReceiver.h"
using RFeatures::ViewReceiver;
#include <cassert>
#include <iostream>
using std::cerr;
using std::endl;


struct ViewReceiver::DataChunk
{
    DataChunk( const vector<cv::Point>* pxs, const vector<cv::Vec3d>* lcs)
        : pxls(pxs), locs(lcs) {}

    const vector<cv::Point>* pxls;
    const vector<cv::Vec3d>* locs;
};  // end DataChunk



ViewReceiver::Ptr ViewReceiver::create( ViewBuilder::Ptr vb, double maxPointRange)
{
    return ViewReceiver::Ptr( new ViewReceiver( vb, maxPointRange));
}   // end create



ViewReceiver::ViewReceiver( ViewBuilder::Ptr vb, double mpr)
    : MAX_POINT_RANGE(mpr), viewBuilder_(vb), nxtPxlPos_(0)
{}   // end ctor



// Purge the cache of data!
ViewReceiver::~ViewReceiver()
{
    BOOST_FOREACH ( DataChunk* &dc, cache_)
    {
        if ( dc != NULL)
        {
            delete dc->pxls;
            delete dc->locs;
            delete dc;
            dc = NULL;
        }   // end if
    }   // end foreach
}   // end dtor



PointCloud::Ptr ViewReceiver::receivePointData( const vector<cv::Point> *pxlsPtr, const vector<cv::Vec3d> *locsPtr)
{
    assert( pxlsPtr != NULL && locsPtr != NULL);
    assert( pxlsPtr->size() == locsPtr->size());

    const View::Ptr v = viewBuilder_->getView();
    const cv::Size imgSz = v->img2d.size();
    const cv::Vec3d vPos = v->posVec;
    const cv::Vec3d fVec = v->focalVec;
    const cv::Mat_<cv::Vec3b> img = v->img2d;

    PointCloud::Ptr newPoints = PointCloud::create();
    while ( checkCache( pxlsPtr, locsPtr))    // Finds in-order set of points (if any)
    {
        // Alias for convenience
        const vector<cv::Point> &pxls = *pxlsPtr;
        const vector<cv::Vec3d> &locs = *locsPtr;

        const int sz = pxls.size();
        for ( int i = 0; i < sz; ++i)
        {
            int col = pxls[i].x;
            int row = pxls[i].y;

            cv::Vec3d rp(0,0,0);    // Relative point
            double range = 0;
            byte bb = 0;    // Blue
            byte gg = 0;    // Green
            byte rr = 0;    // Red

            if ( locs[i][0] != 0.0 && locs[i][1] != 0.0 && locs[i][2] != 0.0)
            {
                //rp = locs[i] - vPos; // Relative point
                // Rectify to align with x,y,z coords (depth as z)
                rp = ViewReceiver::rectify90Point( locs[i], col, row, imgSz, vPos, fVec);

                // Ignore the point if too distant from the view's location
                if ( rp[2] > MAX_POINT_RANGE)
                    rp = cv::Vec3d(0,0,0);
                else
                {
                    //range = fVec.dot( rp) / cv::norm(fVec);
                    range = rp[2];
                    const cv::Vec3b &pxl = img(row,col);
                    bb = pxl[2];
                    gg = pxl[1];
                    rr = pxl[0];
                    newPoints->add(rp[0], rp[1], rp[2], bb, gg, rr);
                }   // end else
            }   // end if

            viewBuilder_->setPoint( row, col, rp[0], rp[1], rp[2], range, bb, gg, rr);
        }   // end for

        delete pxlsPtr;
        delete locsPtr;
        pxlsPtr = NULL;
        locsPtr = NULL;
    }   // end while

    return newPoints;
}   // end receivePointData



bool ViewReceiver::isFinished() const
{
    assert( !viewBuilder_->isViewReady() || cache_.empty());
    return viewBuilder_->isViewReady();
}   // end isFinished



double ViewReceiver::getProportionReceived() const
{
    const cv::Size sz = viewBuilder_->getView()->img2d.size();
    const int szCnt = sz.width * sz.height;
    if ( szCnt == 0)
        return 0;
    return (double)nxtPxlPos_ / szCnt;
}   // end getProportionReceived



bool ViewReceiver::checkCache( const vector<cv::Point>* &pxls, const vector<cv::Vec3d>* &locs)
{
    const int COLS = viewBuilder_->getView()->img2d.cols; // Width of the image

    // Store data in cache and check for next in-order data (if any).
    // We store the new data at the front because the newest data is most likely
    // to be the in-order data and we want to avoid searching the whole cache if possible.
    if ( pxls != NULL && locs != NULL)
    {   // Only add non-NULL points to the cache
        DataChunk* dc = new DataChunk( pxls, locs);
        cache_.push_front(dc);
    }   // end if

    bool found = false;
    // Search the cache for an in order entry
    BOOST_FOREACH ( DataChunk* &dc, cache_)
    {
        const cv::Point &fpx = *dc->pxls->begin();  // First pixel for this chunk
        int thisPxlPos = fpx.y * COLS + fpx.x;

        assert( thisPxlPos >= nxtPxlPos_);

        if ( thisPxlPos == nxtPxlPos_)
        {   // Found in-order data in cache
            // Set the pointers in the out parameters and remove them from the cache
            pxls = dc->pxls;
            locs = dc->locs;
            nxtPxlPos_ += pxls->size();
            delete dc;
            cache_.remove( dc);
            found = true;
            break;
        }   // end if
    }   // end foreach

    return found;
}   // end checkCache



// static
cv::Vec3d ViewReceiver::rectify90Point( const cv::Vec3d &p, int x, int y, const cv::Size &viewDims,
                                                 const cv::Vec3d &camPos, const cv::Vec3d &focus)
                                                 throw (rlib::InvalidVectorException)
{
    assert( viewDims.width == viewDims.height);  // Only works for square images
    const double focLen = (double)viewDims.width / 2;    // VIEW ASSUMED SQUARE & 90 DEGREE FOV
    const double xcentre = (double)viewDims.width / 2;
    const double ycentre = (double)viewDims.height / 2;

    cv::Vec3d pos;    // Will be the rectified point in space
    const cv::Vec3d pt = p - camPos; // Point is specified relative to camera position
    // Rectify the 3D coordinate to match with the respective 2D image pixel
    pos[2] = focus.dot(pt); // Depth (Z) in local frame
    if ( pos[2] < 0)     // Negative depth indicates that the focal vector is way off
    {
        cerr << "ERROR: focus (x,y,z) = " << focus[0] << ", " << focus[1] << ", " << focus[2] << endl;
        cerr << "ERROR: point (x,y,z) = " << pt[0] << ", " << pt[1] << ", " << pt[2] << endl;
        cerr << "ERROR: cam (x,y,z) = " << camPos[0] << ", " << camPos[1] << ", " << camPos[2] << endl;
        cerr << "ERROR: opoint (x,y,z) = " << p[0] << ", " << p[1] << ", " << p[2] << endl;

        throw rlib::InvalidVectorException( "Focal vector does not point into the view!");
    }   // end if
    pos[0] = pos[2] * (xcentre - x) / focLen;
    pos[1] = pos[2] * (ycentre - y) / focLen;

    return pos;
}    // end rectify90Point
