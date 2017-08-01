#include "FeatureUtils.h"
#include "DepthSegmenter.h"
using namespace RFeatures;
#include <cstdlib>
#include <cassert>
#include <iostream>
using std::cout;
using std::cerr;
using std::endl;
#include <ios>
using std::ios;
#include <cmath>
#include <sstream>
using std::ostringstream;
#include <vector>
using std::vector;
#include <fstream>
#include <algorithm>
#include <Random.h> // rlib


double RFeatures::calcTriangleArea( const cv::Vec3f& v0, const cv::Vec3f& v1, const cv::Vec3f& v2)
{
    const double a = cv::norm( v0 - v1);
    const double b = cv::norm( v0 - v2);
    const double c = cv::norm( v2 - v1);
    return calcTriangleArea( a, b, c);
}   // end calcTriangleArea


double RFeatures::calcTriangleArea( const cv::Vec3d& v0, const cv::Vec3d& v1, const cv::Vec3d& v2)
{
    const double a = cv::norm( v0 - v1);
    const double b = cv::norm( v0 - v2);
    const double c = cv::norm( v2 - v1);
    return calcTriangleArea( a, b, c);
}   // end calcTriangleArea


double RFeatures::calcTriangleArea( double a, double b, double c)
{
    double s = (a+b+c)/2;
    return sqrt( s*(s-a)*(s-b)*(s-c));
}   // end calcTriangleArea


cv::Rect RFeatures::calcRelativeRect( const cv::Rect& r0, const cv::Size& s0, const cv::Size& s1)
{
    if ( s0 == s1)
        return r0;

    return cv::Rect( (int)( (double(r0.x) / s0.width) * s1.width),
                     (int)( (double(r0.y) / s0.height) * s1.height),
                     (int)( (double(r0.width) / s0.width) * s1.width),
                     (int)( (double(r0.height) / s0.height) * s1.height));
}   // end calcRelativeRect



cv::Rect RFeatures::createRandomRect( const cv::Size &bounds, const cv::Size &minSize, rlib::Random* rndGen)
{
    cv::Size minSz = minSize;
    if ( minSz.width < 1)
        minSz.width = 1;
    if ( minSz.height < 1)
        minSz.height = 1;

    const int xrange = bounds.width - minSz.width;
    const int yrange = bounds.height - minSz.height;
    if ( xrange < 0 || yrange < 0)
        return cv::Rect(0,0,0,0);

    const int x = rndGen->getRandomInt() % (xrange+1);
    const int wrng = bounds.width - (x + minSize.width);
    const int w = rndGen->getRandomInt() % (wrng+1) + minSize.width;

    const int y = rndGen->getRandomInt() % (yrange+1);
    const int hrng = bounds.height - (y + minSize.height);
    const int h = rndGen->getRandomInt() % (hrng+1) + minSize.height;

    assert( w >= minSize.width);
    assert( h >= minSize.height);

    const cv::Rect rndRect( x, y, w, h);
    const cv::Rect contRect(0,0,bounds.width,bounds.height);
    assert( (contRect & rndRect) == rndRect);
    return rndRect;
}   // end createRandomRect



cv::Rect& RFeatures::scale( cv::Rect& rct, float sf)
{
    rct.x = int(rct.x * sf);
    rct.y = int(rct.y * sf);
    rct.width = int(rct.width * sf);
    rct.height = int(rct.height * sf);
    return rct;
}   // end scale


cv::Point RFeatures::calcPixelOffset( const cv::Rect& r, const cv::Point2f& offset)
{
    return cv::Point( r.x + (int)cvRound(offset.x*r.width), r.y + (int)cvRound(offset.y*r.height));
}   // end calcPixelOffset


cv::Point2f RFeatures::calcOffset( const cv::Rect_<float>& r, const cv::Point2f& offset)
{
    return cv::Point2f( r.x + offset.x*r.width, r.y + offset.y*r.height);
}   // end calcOffset


cv::Point2f RFeatures::calcOffset( const cv::RotatedRect& r, const cv::Point2f& offset)
{
    // Get offset to the top left from the centre point
    const cv::Vec2f topLeftVec( -r.size.width/2, -r.size.height/2);
    const cv::Vec2f offsetVec( offset.x * r.size.width, offset.y * r.size.height);
    // Calculate the new position vector relative to the centre of unrotated r
    const cv::Vec2f posVec = topLeftVec + offsetVec;
    const float posVecMag = (float)cv::norm(posVec);
    // Get the required angle (actual angle of posVec plus the rotated rectangle's angle)
    const float reqRads = float(atan2f( posVec[1], posVec[0]) + r.angle*CV_PI/180);
    const cv::Vec2f nposVec( posVecMag * cosf(reqRads), posVecMag * sinf(reqRads));
    // Add to the centre to get new offset position
    return cv::Point2f( r.center.x + nposVec[0], r.center.y + nposVec[1]);
}   // end calcOffset


cv::Point RFeatures::calcPixelCentre( const cv::Rect& r)
{
    return calcPixelOffset( r, cv::Point2f(0.5,0.5));
}   // end calcPixelCentre


int RFeatures::nonZeroToPoints( const cv::Mat_<byte>& m, std::vector<cv::Point>& pts)
{
    int numAdded = 0;
    const int rows = m.rows;
    const int cols = m.cols;
    for ( int i = 0; i < rows; ++i)
    {
        const byte* mrow = m.ptr<byte>(i);
        for ( int j = 0; j < cols; ++j)
        {
            if ( mrow[j])
            {
                pts.push_back(cv::Point(j,i));
                numAdded++;
            }   // end if
        }   // end for - cols
    }   // end for - rows
    return numAdded;
}   // end nonZeroToPoints


int RFeatures::findBoundingBox( const cv::Mat_<byte>& m, cv::Rect& bbox)
{
    int num = 0;
    const int rows = m.rows;
    const int cols = m.cols;
    bbox = cv::Rect(0,0,0,0);
    for ( int i = 0; i < rows; ++i)
    {
        const byte* mrow = m.ptr<byte>(i);
        for ( int j = 0; j < cols; ++j)
        {
            if ( mrow[j])
            {
                if ( num == 0)
                    bbox = cv::Rect(j,i,1,1);
                else
                    bbox |= cv::Rect(j,i,1,1);
                num++;
            }   // end if
        }   // end for - cols
    }   // end for - rows
    return num;
}   // end findBoundingBox


cv::Mat_<byte> RFeatures::pointsToMask( const cv::Size& sz, const std::vector<cv::Point>& pts)
{
    cv::Mat_<byte> mask = cv::Mat_<byte>::zeros(sz);
    BOOST_FOREACH ( const cv::Point& p, pts)
        mask.at<byte>(p) = 255;
    return mask;
}   // end pointsToMask



/******************** DepthAgnosticImg *******************/
RFeatures::DepthAgnosticImg::DepthAgnosticImg( const cv::Mat& img)
    : _img(img), _imgArr( img.ptr()), _rows( (int)img.rows), _cols( (int)img.cols),
    _depth( (int)img.depth()), _elemSz( (int)img.elemSize1()), _imgRct(0,0,img.cols,img.rows)
{
    assert( img.channels() == 1);
}   // end ctor


double RFeatures::DepthAgnosticImg::operator()( const cv::Point& p) const
{
    assert( _imgRct.contains(p));
    const byte* vptr = &_img.ptr(p.y)[_elemSz*p.x];
    return RFeatures::pval( _depth, vptr);
}   // end operator()
/*****************End DepthAgnosticImg *******************/



bool RFeatures::isWithin( const cv::Rect& outer, const cv::Rect& inner)
{
    return outer.contains( cv::Point(inner.x,                   inner.y)) &&
           outer.contains( cv::Point(inner.x + inner.width - 1, inner.y)) &&
           outer.contains( cv::Point(inner.x + inner.width - 1, inner.y + inner.height - 1)) &&
           outer.contains( cv::Point(inner.x,                   inner.y + inner.height - 1));
}   // end isWithin
bool RFeatures::isContained( const cv::Rect& outer, const cv::Rect& inner) { return isWithin(outer,inner);}


bool RFeatures::isWithin( const cv::Mat& m, const cv::Rect& inner)
{
    const cv::Rect imgRect(0,0,m.cols,m.rows);
    return isWithin( imgRect, inner);
}   // end isWithin
bool RFeatures::isContained( const cv::Mat& m, const cv::Rect& inner) { return isWithin(m,inner);}


bool RFeatures::isWithin( const cv::Mat& m, const cv::Point& inner)
{
    return cv::Rect(0,0,m.cols,m.rows).contains(inner);
}   // end isWithin
bool RFeatures::isContained( const cv::Mat& m, const cv::Point& inner) { return isWithin(m,inner);}



cv::Point RFeatures::findLocalMin( const cv::Mat& img, const cv::Point& sp, const cv::Point& p0, const cv::Point& p1)
{
    const RFeatures::DepthAgnosticImg aimg(img);

    const int xdiff = p1.x - p0.x;
    const int ydiff = p1.y - p0.y;
    const int npsteps = std::max<int>( abs(xdiff), abs(ydiff));
    const cv::Rect pBounds( std::min<int>(p0.x,p1.x), std::min<int>(p0.y,p1.y), abs(xdiff)+1, abs(ydiff)+1);
    assert( isWithin( img, pBounds));

    // Calc per pixel increment in x and y directions
    const double xdelta = double(xdiff)/npsteps;
    const double ydelta = double(ydiff)/npsteps;

    // Go from sp towards p1 trying to minimise local value first
    double mval = DBL_MAX;    // Min value
    cv::Point mp = sp;  // Min point

    int pstep = 0;
    cv::Point p = sp;   // Start point
    assert( pBounds.contains(p));
    double v = aimg( p);
    while ( v < mval)
    {
        mval = v;
        mp = p;

        pstep++;
        p.x = sp.x + int(pstep*xdelta);
        p.y = sp.y + int(pstep*ydelta);
        if ( !pBounds.contains(p))
            break;
        v = aimg( p);
    }   // end while

    // Go in the other direction to find the local minimum
    pstep = 0;
    p.x = sp.x - int(pstep*xdelta);
    p.y = sp.y - int(pstep*ydelta);
    assert( pBounds.contains(p));
    v = aimg( p);
    while ( v < mval)
    {
        mval = v;
        mp = p;

        pstep++;
        p.x = sp.x - int(pstep*xdelta);
        p.y = sp.y - int(pstep*ydelta);
        if ( !pBounds.contains(p))
            break;
        v = aimg( p);
    }   // end while

    return mp;
}   // end findLocalMin


cv::Point RFeatures::findMin( const cv::Mat_<float>& img, const cv::Point& p0, const cv::Point& p1, float* minVal)
{
    const int xdiff = p1.x - p0.x;
    const int ydiff = p1.y - p0.y;
    const int npsteps = std::max<int>( abs(xdiff), abs(ydiff));

    // Calc per pixel increment in x and y directions
    const double xdelta = double(xdiff)/npsteps;
    const double ydelta = double(ydiff)/npsteps;

    // Go from sp in one direction trying to minimise local value first
    double mval = DBL_MAX;    // Min value
    cv::Point mp = p0;  // Min point

    cv::Point p;
    double v;
    for ( int i = 0; i < npsteps; ++i)
    {
        p.x = p0.x + cvRound(i*xdelta);
        p.y = p0.y + cvRound(i*ydelta);
        v = img.at<float>( p);
        if ( v < mval)
        {
            mval = v;
            mp = p;
        }   // end if
    }   // end for

    if ( minVal != NULL)
        *minVal = (float)mval;

    return mp;
}   // end findMin


float RFeatures::findOrthogonalMaxDelta( const cv::Mat_<float>& dmap, const cv::Point& p0, const cv::Point& p1, cv::Point& dpoint)
{
    // Calculate the base vector between which we find the orthogonal maximum difference in depth
    const cv::Vec3f v0( (float)p0.x, (float)p0.y, dmap.at<float>(p0));
    const cv::Vec3f v1( (float)p1.x, (float)p1.y, dmap.at<float>(p1));
    float bvecMag = (float)cv::norm( v1 - v0); // Length of base of triangle

    const int xdiff = p1.x - p0.x;
    const int ydiff = p1.y - p0.y;
    const int npsteps = std::max<int>( abs(xdiff), abs(ydiff));

    // Calc per pixel increment in x and y directions
    const float xdelta = float(xdiff)/npsteps;
    const float ydelta = float(ydiff)/npsteps;

    float dmax = 0;      // Max depth value
    cv::Vec3f vmax = v0; // Current point of maximum depth difference

    cv::Vec3f vp;
    float a, b, c, p, areaTriangle, orthDepth;
    for ( int i = 0; i < npsteps; ++i)
    {
        vp[0] = v0[0] + i*xdelta;
        vp[1] = v0[1] + i*ydelta;
        vp[2] = dmap.at<float>( (int)cvRound(vp[1]), (int)cvRound(vp[0]));

        // Calculate depth by way of calculate triangular area using Heron's formula
        a = bvecMag;
        b = (float)cv::norm(vp);
        c = (float)cv::norm(v1 - vp);
        p = (a+b+c)/2;    // Half perimiter
        areaTriangle = sqrtf( p*(p-a)*(p-b)*(p-c));   // Heron's
        // Area of rectangle is bvecMag * depth = 2*areaTriangle
        orthDepth = 2*areaTriangle/bvecMag;

        if ( orthDepth >= dmax)
        {
            dmax = orthDepth;
            vmax = vp;
        }   // end if
    }   // end for

    dpoint.x = (int)cvRound(vp[0]);
    dpoint.y = (int)cvRound(vp[1]);
    return dmax;
}   // end findOrthogonalMaxDelta



bool RFeatures::findMeanPosition( cv::Point& p, const cv::Mat_<byte> mask)
{
    const int rows = mask.rows;
    const int cols = mask.cols;
    int pcount = 0;
    p.x = 0; p.y = 0;
    for ( int i = 0; i < rows; ++i)
    {
        const byte* mrow = mask.ptr<byte>(i);
        for ( int j = 0; j < cols; ++j)
        {
            if ( mrow[j])
            {
                p.x += j;
                p.y += i;
                pcount++;
            }   // end if
        }   // end for - cols
    }   // end for - rows

    if ( pcount)
    {
        p.x = cvRound( float(p.x)/pcount);
        p.y = cvRound( float(p.y)/pcount);
    }   // end if

    return pcount > 0;
}   // end findMeanPosition




cv::Point RFeatures::findIntersectionPoint( const cv::Mat_<byte>& m, const cv::Point& p1, const cv::Point& p2)
{
    const cv::Rect imgRct( 0, 0, m.cols, m.rows);
    const int xdiff = p2.x - p1.x;
    const int ydiff = p2.y - p1.y;
    const int nsteps = std::max<int>( abs(xdiff), abs(ydiff));
    const double xdelta = double(xdiff)/nsteps;
    const double ydelta = double(ydiff)/nsteps;

    for ( int i = 0; i < nsteps; ++i)
    {
        const cv::Point p( p1.x + int(i*xdelta), p1.y + int(i*ydelta));
        if ( !imgRct.contains(p))   // Stop if p falls outside the image bounds
            break;

        if ( m.at<byte>( p))
            return p;
    }   // end for

    return cv::Point(-1,-1);
}   // end findIntersectionPoint




cv::Point RFeatures::findOuterIntersectionPoint( const cv::Mat_<byte>& m, const cv::Point& p1, const cv::Point& p2)
{
    const cv::Rect imgRct( 0, 0, m.cols, m.rows);
    const int xdiff = p2.x - p1.x;
    const int ydiff = p2.y - p1.y;
    const int nsteps = std::max<int>( abs(xdiff), abs(ydiff));
    const double xdelta = double(xdiff)/nsteps;
    const double ydelta = double(ydiff)/nsteps;

    bool foundBoundary = false;
    cv::Point previousPoint;
    for ( int i = 0; i < nsteps; ++i)
    {
        const cv::Point p( p1.x + int(i*xdelta), p1.y + int(i*ydelta));
        if ( !imgRct.contains(p))   // Stop if p falls outside the image bounds
            break;

        if ( foundBoundary && !m.at<byte>( p))
            return previousPoint;

        if ( m.at<byte>( p))
        {
            foundBoundary = true;
            previousPoint = p;
        }   // end if
    }   // end for

    return cv::Point(-1,-1);
}   // end findOuterIntersectionPoint



void RFeatures::setMasked( cv::Mat img, const cv::Mat_<byte> mask)
{
    assert( img.size() == mask.size());
    const int elemSz = (int)img.elemSize();   // Number of bytes to zero out at each image position
    const int rows = mask.rows;
    const int cols = mask.cols;

    for ( int i = 0; i < rows; ++i)
    {
        const byte* mrow = mask.ptr<byte>(i);
        byte* outRow = img.ptr(i);

        for ( int j = 0; j < cols; ++j)
        {
            if ( mrow[j] == 0)
                memset( &outRow[j*elemSz], 0, elemSz);
        }   // end for - cols
    }   // end for - rows
}   // end setMasked


void RFeatures::drawConvexHull( const vector<cv::Point>& pts, cv::Mat img, cv::Scalar col, bool filled)
{
    vector<cv::Point> hullpts;
    cv::convexHull( pts, hullpts);
    if ( filled)
        cv::fillConvexPoly( img, &hullpts[0], (int)hullpts.size(), col);
    else
        drawPoly( hullpts, img, col);
}   // end drawConvexHull


void RFeatures::drawFilledPoly( const vector<cv::Point>& pts, cv::Mat img, cv::Scalar col)
{
    int numArr = (int)pts.size();
    const cv::Point* ptsArr = &pts[0];
    cv::fillPoly( img, &ptsArr, &numArr, 1/*numContours*/, col);
}   // end drawFilledPoly


void RFeatures::drawPoly( const vector<cv::Point>& pts, cv::Mat img, cv::Scalar col)
{
    const int numArr = (int)pts.size();
    const cv::Point* ptsArr = &pts[0];
    cv::polylines( img, &ptsArr, &numArr, 1/*numContours*/, true/*isClosed*/, col, 1);
}   // end drawPoly


void RFeatures::createHistogram( const cv::Mat_<byte>& m, vector<int>& hist)
{
    hist.clear();
    hist.resize(256);   // All entries zero'd
    const int nrows = m.rows;
    const int ncols = m.cols;
    for ( int i = 0; i < nrows; ++i)
    {
        const byte* mrow = m.ptr<byte>(i);
        for ( int j = 0; j < ncols; ++j)
            hist[mrow[j]]++;
    }   // end for - rows
}   // end createHistogram



cv::Mat_<cv::Vec3b> RFeatures::drawHistogram( const vector<int>& hist, const cv::Size& imgSz, bool makeProbDist)
{
    const int numBins = (int)hist.size();
    int baseLen = cvRound(float(imgSz.width)/numBins);
    if ( baseLen < 1)
        baseLen = 1;

    const cv::Size sz( baseLen * numBins, imgSz.height);
    cv::Mat_<cv::Vec3b> hgraph = cv::Mat_<cv::Vec3b>::zeros( sz);

    int sumVal = 0;
    int maxVal = 0;
    for ( int i = 0; i < numBins; ++i)
    {
        sumVal += hist[i];
        if ( hist[i] > maxVal)
            maxVal = hist[i];
    }   // end for

    rlib::Random rndGen;
    cv::Rect bar(0,0,baseLen,0);
    for ( int i = 0; i < numBins; ++i)
    {
        float prop = float(hist[i])/maxVal;
        if ( makeProbDist)
            prop = float(hist[i])/sumVal;

        bar.height = cvRound(sz.height * prop);
        if ( bar.height > hgraph.rows)
            bar.height = hgraph.rows;

        bar.y = hgraph.rows - bar.height;
        cv::Scalar colour(150,150,150,150);
        createRandomColour( colour, &rndGen);
        cv::rectangle( hgraph, bar, colour, -1); // Filled
        bar.x += baseLen;   // For next bar
    }   // end for

    return hgraph;
}   // end drawHistogram



cv::Mat_<float> RFeatures::make2DGaussian( const cv::Size filterSz, double xcentre, double ycentre)
{
    cv::Mat_<float> gfilter( filterSz);
    const int rows = filterSz.height;
    const int cols = filterSz.width;

    const double ax = -xcentre * (cols-1);
    const double ay = -ycentre * (rows-1);

    const float sigma2 = float(cols*rows)/2;
    //const double sigma2 = 2;
    double sum = 0;
    for ( int y = 0; y < rows; ++y)
    {
        const float yterm = powf(ay+y,2.0f);
        float* g_row = gfilter.ptr<float>(y);
        for ( int x = 0; x < cols; ++x)
        {
            g_row[x] = expf( -( powf(ax+x,2.0f) + yterm)/sigma2);
            sum += g_row[x];
        }   // end for - cols
    }   // end for - rows

    gfilter *= 1.0/sum; // Normalise
    return gfilter;
}   // end make2DGaussian



cv::Mat_<float> RFeatures::toRowVectors( const cv::Mat& img, double alpha, double beta)
{
    vector<cv::Mat> channels;
    cv::split( img, channels);

    cv::Mat_<float> fvs( 0, img.rows * img.cols);
    BOOST_FOREACH ( const cv::Mat& m, channels)
    {
        cv::Mat n;
        m.convertTo( n, CV_32F, alpha, beta);    // Convert to float with optional scaling
        fvs.push_back( n.reshape(0, 1));   // Make single row
    }   // end for

    return fvs;
}   // end toRowVectors



cv::Mat_<float> RFeatures::toRowVector( const cv::Mat& img, double alpha, double beta)
{
    return toRowVectors(img, alpha, beta).reshape(0,1);// Make single row
}   // end toRowVector



ostream& RFeatures::operator<<( ostream& os, const cv::Rect& r)
{
    return os << "[" << r.x << " " << r.y << " " << r.width << " " << r.height << "]";
}   // end operator<<


istream& RFeatures::operator>>( istream& is, cv::Rect& r)
{
    char ch;
    return is >> ch >> r.x >> r.y >> r.width >> r.height >> ch;
}   // end operator>>


ostream& RFeatures::operator<<( ostream& os, const cv::Size& s)
{
    return os << s.width << " " << s.height;
}   // end operator<<


istream& RFeatures::operator>>( istream& is, cv::Size& s)
{
    return is >> s.width >> s.height;
}   // end operator>>



void RFeatures::sqrtGammaCorrect( cv::Mat &img, int MAX_VAL, int row, int col)
{
    int newGreyVal = (int)( sqrt(img.ptr<const uchar>(row)[col] * MAX_VAL) + 0.5);
    img.ptr<uchar>(row)[col] = (uchar)newGreyVal;
}   // end sqrtGammaCorrect



cv::Mat RFeatures::sqrtGammaCorrect( const cv::Mat& img)
{
    assert( img.depth() == CV_8U);
    int rows = img.rows;
    int nc = img.channels() * img.cols;
    cv::Mat cimg( img.size(), img.type());
    assert( cimg.isContinuous());

    if ( img.isContinuous())
    {
        nc *= rows;
        rows = 1;
    }   // end if

    const double gamVal = 255;

    for ( int i = 0; i < rows; ++i)
    {
        const byte* inPtr = img.ptr<byte>(i);
        byte* outPtr = cimg.ptr<byte>(i);
        for ( int j = 0; j < nc; ++j)
            *outPtr++ = byte(int(sqrt( double(*inPtr++) * gamVal) + 0.5));
    }   // end for - rows

    return cimg;
}   // end sqrtGammaCorrect



string RFeatures::imgDepthToString( const cv::Mat &img)
{
    return imgDepthToString( img.depth());
}   // end imgDepthToString



string RFeatures::imgDepthToString( int depth)
{
    string dstr;

    switch( depth)
    {
        case CV_8U: dstr = "CV_8U"; break;
        case CV_8S: dstr = "CV_8S"; break;
        case CV_16U: dstr = "CV_16U"; break;
        case CV_16S: dstr = "CV_16S"; break;
        case CV_32F: dstr = "CV_32F"; break;
        case CV_32S: dstr = "CV_32S"; break;
        case CV_64F: dstr = "CV_64F"; break;
    }   // end switch

    return dstr;
}   // end imgDepthToString



string RFeatures::imgTypeToString( const cv::Mat &img)
{
    ostringstream oss;
    oss << imgDepthToString( img.depth()) << "C" << img.channels();
    return oss.str();
}   // end imgTypeToString



ostream& RFeatures::writeBinary( ostream &os, const cv::Mat &m)
{
    int params[4] = {m.rows, m.cols, m.channels(), m.depth()};
    os.write( (const char*)params, sizeof(params));
    int rowLen = m.cols*int(m.elemSize());   // Row length in bytes
    for ( int i = 0; i < m.rows; ++i)
        os.write( (const char*)m.ptr<const char>(i), rowLen);
    return os;
}   // end writeBinary


bool RFeatures::saveBinaryImage( const string fname, const cv::Mat& m)
{
    std::ofstream ofs( fname.c_str());
    if ( ofs.good())
        writeBinary( ofs, m);
    ofs.flush();
    const bool saved = ofs.good();
    ofs.close();
    return saved;
}   // end saveBinaryImage



istream& RFeatures::readBinary( istream &is, cv::Mat &m)
{
    int params[4];
    int rows, cols, channels, depth;
    is.read( (char*)params, sizeof(params));
    rows = params[0];
    cols = params[1];
    channels = params[2];
    depth = params[3];

    int mtype = -1;   // Type of matrix
    switch ( depth)
    {
        case CV_8U: mtype = CV_8UC(channels); break;
        case CV_8S: mtype = CV_8SC(channels); break;
        case CV_16U: mtype = CV_16UC(channels); break;
        case CV_16S: mtype = CV_16SC(channels); break;
        case CV_32F: mtype = CV_32FC(channels); break;
        case CV_32S: mtype = CV_32SC(channels); break;
        case CV_64F: mtype = CV_64FC(channels); break;
    }   // end switch
    assert( mtype != -1);

    cv::Mat tmp( rows, cols, mtype);   // Temporary just to be able to use elemSize()
    const int rowBytes = cols*int(tmp.elemSize());   // Bytes per row

    char* dataBuff = (char*)cv::fastMalloc( rows * rowBytes * sizeof(char));    // Data buffer
    is.read( dataBuff, rows * rowBytes * sizeof(char));
    tmp = cv::Mat( rows, cols, mtype, (void*)dataBuff);
    m = tmp.clone();
    cv::fastFree(dataBuff); // Release memory

    return is;
}   // end readBinary



bool RFeatures::loadBinaryImage( const string fname, cv::Mat& m)
{
    bool loaded = false;
    std::ifstream ifs( fname.c_str());
    if ( ifs.good())
    {
        readBinary( ifs, m);
        loaded = true;
        ifs.close();
    }   // end if
    return loaded;
}   // end loadBinaryImage



void RFeatures::showImage( const cv::Mat &img, const string &title, bool wait)
{
    try
    {
        cv::namedWindow( title.c_str());
        cv::imshow( title.c_str(), img);
        if ( wait)
            cv::waitKey();
    }   // end
    catch ( const cv::Exception &e)
    {
        cerr << "ERROR: RFeatures::showImage failed!" << endl;
        cerr << e.what() << endl;
    }   // catch
}   // end showImage



void RFeatures::closeImage( const string& title)
{
    cv::destroyWindow( title);
}   // end closeImage



ostream &RFeatures::print( ostream &os, const cv::Mat &mat)
{
    os << "Width: " << mat.cols << "; Height: " << mat.rows
       << "; Type: " << imgTypeToString( mat)
       << "; Continuous?: " << (mat.isContinuous() ? "true" : "false") << endl;
    return os;
}   // end print



bool RFeatures::loadImage( const string &fname, cv::Mat &img, bool bw)
{
    img = cv::imread( fname, !bw);
    if ( img.empty())
        return false;
    // Image may still be 3 channel even if loaded black and white
    // so we flatten it here if need be.
    if ( bw && img.channels() != 1)
        img = flatten(img);
    return true;
}   // end loadImage



bool RFeatures::saveImage( const string &fname, const cv::Mat &img)
{
    return cv::imwrite( fname, img);
}   // end saveImage



cv::Mat RFeatures::convertForDisplay( const cv::Mat &img, bool forceScale)
{
    assert( img.channels() == 1 || img.channels() == 3);

    // Scale values within [0,255] if necessary
    // Must first reshape matrix since minMaxLoc can only work on single channel matrices
    cv::Mat himg = img.reshape(1);
    double mn, mx;
    cv::minMaxLoc( himg, &mn, &mx);
    //cerr << "First min, max values: " << mn << ", " << mx << endl;

    cv::Mat nimg = img;
    if ( forceScale || mn < 0.0 || mx > 255.0)
        nimg = rescale( img, 0, 255);

    cv::minMaxLoc( nimg, &mn, &mx);
    //cerr << "Second min, max values: " << mn << ", " << mx << endl;

    cv::Mat dimg;
    nimg.convertTo( dimg, CV_8U);
    return dimg;
}   // end convertForDisplay



cv::Mat_<cv::Vec3b> RFeatures::convertFromSingleChannel( const cv::Mat& rimg, bool invert)
{
    assert(rimg.channels()==1);
    double mn, mx;
    cv::minMaxLoc( rimg, &mn, &mx);

    cv::Mat bmat;
    if ( invert)
        rimg.convertTo( bmat, CV_8U, -255./mx, 255);
    else
        rimg.convertTo( bmat, CV_8U, 255./mx);

    cv::Mat barr[3] = {bmat,bmat,bmat};
    cv::Mat omat;
    cv::merge( barr, 3, omat);
    return omat;
}   // end convertFromSingleChannel



cv::Mat RFeatures::rescale( const cv::Mat img, double minVal, double maxVal)
{
    assert( maxVal > minVal);

    const cv::Mat timg = img.clone();
    // Image must be reshaped to use only a single channel to find the
    // min and max value over all channels.
    cv::Mat himg = timg.reshape(1);
    double mn, mx;
    cv::minMaxLoc( himg, &mn, &mx);

    double scale = (maxVal - minVal) / (mx - mn);   // Scale factor
    return scale * (img - mn); // Scale the values
}   // end rescale



cv::Mat_<byte> RFeatures::contrastStretch( const cv::Mat& m, const cv::Mat_<byte> mask)
{
    double mn, mx;
    cv::minMaxLoc( m, &mn, &mx, 0, 0, mask);
    cv::Mat_<byte> outm( m.size());
    const cv::Mat mtemp = m - mn;
    mtemp.convertTo( outm, CV_8U, 255./(mx-mn));
    return outm;
}   // end contrastStretch



cv::Mat_<float> RFeatures::truncateAndScale( const cv::Mat_<float> img, float trunc, float scale)
{
    const cv::Mat_<byte> mask = img > trunc;
    // Values in img more than trunc will be set to trunc
    cv::Mat_<float> fimg = img.clone();
    float* vals = fimg.ptr<float>();
    const int nvals = (int)fimg.total();
    const float scaleFactor = scale/trunc;
    for ( int i = 0; i < nvals; ++i)
        vals[i] = std::min<float>( vals[i], trunc) * scaleFactor;

    return fimg;
}   // end truncateAndScale



cv::Mat_<cv::Vec3b> RFeatures::makeCV_8UC3( const cv::Mat_<float> rmap)
{
    double mn, mx;
    cv::minMaxLoc( rmap, &mn, &mx);
    cv::Mat_<byte> bmap;
    rmap.convertTo( bmap, CV_8U, 255./mx);
    cv::Mat_<byte> channels[3] = {bmap, bmap, bmap};
    cv::Mat_<cv::Vec3b> omap( rmap.size());
    cv::merge( channels, 3, omap);
    return omap;
}   // end makeCV_8UC3



cv::Mat RFeatures::scale( const cv::Mat img, const cv::Size& newSz)
{
    if ( img.rows == newSz.height && img.cols == newSz.width)
        return img;
    cv::Mat m;
    cv::resize( img, m, newSz);
    return m;
}   // end scale



cv::Mat RFeatures::flatten( const cv::Mat &img)
{
    int ndims = img.channels();
    // Convert img to double (so division of channel values is accurate)
    cv::Mat dimg;
    img.convertTo( dimg, CV_64F);

    vector<cv::Mat> planes;
    cv::split( dimg, planes);

    // Result is a single channel of the same type (CV_64F)
    cv::Mat result( dimg.rows, dimg.cols, dimg.depth(), cv::Scalar(0));
    BOOST_FOREACH( cv::Mat plane, planes)
        result += plane / ndims;

    // Convert back to original type if need be
    if (result.depth() != img.depth())
        result.convertTo( result, img.depth());

    return result;
}   // end flatten



cv::Mat RFeatures::convolve( const cv::Mat &map, const cv::Mat &krn)
{
    assert( krn.channels() == map.channels());

    cv::Mat rsp( map.rows, map.cols, CV_64F, cv::Scalar(0)); // Output response

    // Split the image planes (dimensions) of the provided map and kernel
    vector<cv::Mat> planes;
    cv::split( map, planes);
    vector<cv::Mat> kernels;
    cv::split( krn, kernels);

    int ndims = (int)planes.size();  // Number of dimensions
    for ( int n = 0; n < ndims; ++n)
    {
        cv::Mat rn;   // Response for this dimension
        cv::filter2D( planes[n], rn, CV_64F, kernels[n]);  // DFT based
        rsp += rn; // Add this response to the matrix of response scalars
    }   // end for

    return rsp;
}   // end convolve



void RFeatures::showScaledPlanes( const cv::Mat &img, const string &winNamePrefix)
{
    int ndims = img.channels();
    vector<cv::Mat> planes;
    cv::split(img, planes);

    for ( int i = 0; i < ndims; ++i)
    {
        cv::Mat displayImg = convertForDisplay( planes[i]);
        // Display the window
        ostringstream oss;
        oss << winNamePrefix << " (plane: " << i << ")";
        showImage( displayImg, oss.str(), true);
    }   // end for
}   // end showScaledPlanes



void RFeatures::drawBoxes( cv::Mat &img, const vector<cv::Rect> &boxes, int thick, const cv::Scalar col)
{
    BOOST_FOREACH( cv::Rect box, boxes)
        cv::rectangle( img, box, col, thick);
}   // end drawBoxes


void RFeatures::drawRotatedRect( cv::Mat& img, const cv::RotatedRect& rct, int thick, const cv::Scalar col)
{
    const cv::Size& sz = img.size();
    const int NPTS = 4;
    std::vector<cv::Point2f> pts(NPTS);
    rct.points( &pts[0]);
    for ( int i = 0; i < NPTS; ++i)
    {
        cv::Point p0 = pts[i];
        cv::Point p1 = pts[(i+1) % NPTS];
        if ( cv::clipLine( sz, p0, p1)) // Only need to draw if returns true (part of line segment in image)
            cv::line( img, p0, p1, col, thick);
    }   // end for
}   // end drawRotatedRect


void RFeatures::calcNormalsCrossings( double s1, double s2, double u1, double u2, double &x1, double &x2)
{
    double s1_2 = s1*s1;
    double s2_2 = s2*s2;
    double frontTerm = s1_2*u2 - s2_2*u1;
    double denom = s1_2 - s2_2;
    double rootTerm = sqrt( s1_2*s2_2*( pow(u1-u2,2) + 2*(log(s1) - log(s2))*denom));
    x1 = (frontTerm + rootTerm) / denom;
    x2 = (frontTerm - rootTerm) / denom;
}   // end calcNormalsCrossings


double RFeatures::calcNormal( double s, double u, double x)
{
    static const double SQRT2PI = sqrt(2*CV_PI);
    return exp( -pow(x-u,2) / (2*s*s) ) / (s*SQRT2PI);
}   // end calcNormal


int RFeatures::roundMult( double v, int m)
{
    return (int)(v+(double)m/2) / m * m;
}   // end roundMult


cv::Vec3f RFeatures::project( const cv::Vec3f& p, const cv::Vec3f& base)
{
    const double bnorm = powf(cv::norm(base),2);
    const double baseNum = base.dot(base);
    return p * float(baseNum/bnorm);
}   // end project


double RFeatures::calcSumSqDiffs( const vector<double>& vals, double mean)
{
    double sumSqDiffs = 0;
    BOOST_FOREACH( const double &d, vals)
        sumSqDiffs += pow(d-mean,2);
    return sumSqDiffs;
}   // end calcSumSqDiffs


double RFeatures::calcStdDev( const vector<double> &vals, double mean, int bias)
{
    const double sumSqDiffs = calcSumSqDiffs( vals, mean);
    assert( vals.size() - bias > 0);
    return sqrt( sumSqDiffs / (vals.size()-bias));
}   // end calcStdDev


void RFeatures::vertFlipReplace( vector<cv::Mat> &imgs)
{
    const int sz = (int)imgs.size();
    for ( int i = 0; i < sz; ++i)
    {
        const cv::Mat img = imgs.front();
        cv::Mat fimg;
        cv::flip( img, fimg, 1);    // flip about vertical axis
        imgs.erase( imgs.begin());
        imgs.push_back( fimg);
    }   // end for
}   // end vertFlipReplace


void RFeatures::createRandomColour( cv::Scalar &pxl, rlib::Random* rnd)
{
    pxl[0] += (255.0 - pxl[0]) * rnd->getRandom();
    pxl[1] += (255.0 - pxl[1]) * rnd->getRandom();
    pxl[2] += (255.0 - pxl[2]) * rnd->getRandom();
    pxl[3] += (255.0 - pxl[3]) * rnd->getRandom();
}   // end createRandomColour



cv::Mat RFeatures::modifyDescriptor( const cv::Mat &fv, int type, float alpha)
{
    const cv::Mat fv1 = fv.isContinuous() ? fv : fv.clone();
    cv::Mat fv2;
    fv1.reshape( 1, 1).convertTo( fv2, type, alpha);
    return fv2;
}   // end modifyDescriptor



int RFeatures::writeDescriptor( ostream& os, const cv::Mat& dsc)
{
    const cv::Mat_<float> d = modifyDescriptor( dsc, CV_32F, 1);
    const int cols = d.cols;
    if ( cols > 0)
    {
        const float* dptr = d.ptr<float>(0);
        os << dptr[0];
        for ( int i = 1; i < cols; ++i)
            os << " " << dptr[i];
    }   // end if
    return cols;
}   // end writeDescriptor



cv::Mat_<float> RFeatures::readDescriptors( const string fname, bool asCols) throw (DescriptorLengthException)
{
    std::ifstream ifs( fname.c_str());
    cv::Mat_<float> allVecs;
    int len = -1;
    while ( ifs.good())
    {
        const cv::Mat_<float> vec = RFeatures::readDescriptor( ifs, asCols);
        if ( !vec.empty())
        {
            if ( len == -1)
                len = (int)vec.total();

            if ( vec.total() != len)
                throw DescriptorLengthException( "Descriptor length mismatch!");

            allVecs.push_back(vec);
        }   // end if
    }   // end while
    ifs.close();
    return allVecs;
}   // end readDescriptors



cv::Mat_<float> RFeatures::readDescriptor( istream& is, bool asCols)
{
    string ln;
    if ( is.good())
        std::getline(is, ln);
    cv::Mat_<float> vec;
    if ( !ln.empty())
    {
        std::istringstream iss(ln);
        double v;
        while ( iss >> v)
            vec.push_back(v);

        if ((vec.cols == 1 && !asCols) || (vec.rows == 1 && asCols))
            vec = vec.t();  // transpose
    }   // end if
    return vec;
}   // end readDescriptor



cv::Mat_<float> RFeatures::combine( const cv::Mat_<float>& f1, const cv::Mat_<float>& f2)
{
    // Only allow single row/column feature vectors
    assert( f1.rows == 1 || f1.cols == 1);
    assert( f2.rows == 1 || f2.cols == 1);
    cv::Mat_<float> t1 = f1;
    cv::Mat_<float> t2 = f2;
    if ( t1.rows == 1)
        t1 = f1.t();
    if ( t2.rows == 1)
        t2 = f2.t();

    // t1 and t2 are now column vectors

    cv::Mat_<float> f = t1.clone();
    f.push_back(t2);
    assert( f.total() == f1.total() + f2.total());
    return f.t();   // Row vector output
}   // end combine



// Takes yaw, pitch and roll in degrees
void createYawPitchRollMatrices( double yaw, double pitch, double roll, cv::Matx33d& ym, cv::Matx33d& pm, cv::Matx33d& rm)
{
    yaw *= CV_PI/180;
    pitch *= CV_PI/180;
    roll *= CV_PI/180;

    const double cY = cos(yaw);
    const double cP = cos(pitch);
    const double cR = cos(roll);
    const double sY = sin(yaw);
    const double sP = sin(pitch);
    const double sR = sin(roll);

    ym = cv::Matx33d(  cY, sY, 0,
                      -sY, cY, 0,
                        0,  0, 1);

    pm = cv::Matx33d(   1,  0,  0,
                        0, cP,-sP,
                        0, sP, cP);

    rm = cv::Matx33d(  cR,  0, sR,
                        0,  1,  0,
                      -sR,  0, cR);
}   // end createYawPitchRollMatrices



cv::Vec3d RFeatures::applyYawPitchRoll( const cv::Vec3d& v, double yaw, double pitch, double roll)
{
    cv::Matx33d ym, pm, rm;
    createYawPitchRollMatrices( yaw, pitch, roll, ym, pm, rm);
    return ym * pm * rm * v;
}   // end applyYawPitchRoll



cv::Vec3d RFeatures::applyRollPitchYaw( const cv::Vec3d& v, double yaw, double pitch, double roll)
{
    cv::Matx33d ym, pm, rm;
    createYawPitchRollMatrices( yaw, pitch, roll, ym, pm, rm);
    return rm * pm * ym * v;
}   // end applyRollPitchYaw



cv::Vec3d RFeatures::applyPitchRollYaw( const cv::Vec3d& v, double yaw, double pitch, double roll)
{
    cv::Matx33d ym, pm, rm;
    createYawPitchRollMatrices( yaw, pitch, roll, ym, pm, rm);
    return pm * rm * ym * v;
}   // end applyPitchRollYaw



cv::Vec3d RFeatures::applyRollYawPitch( const cv::Vec3d& v, double yaw, double pitch, double roll)
{
    cv::Matx33d ym, pm, rm;
    createYawPitchRollMatrices( yaw, pitch, roll, ym, pm, rm);
    return rm * ym * pm * v;
}   // end applyRollYawPitch



cv::Vec3d RFeatures::applyPitchYawRoll( const cv::Vec3d& v, double yaw, double pitch, double roll)
{
    cv::Matx33d ym, pm, rm;
    createYawPitchRollMatrices( yaw, pitch, roll, ym, pm, rm);
    return pm * ym * rm * v;
}   // end applyPitchYawRoll



cv::Vec3d RFeatures::applyYawRollPitch( const cv::Vec3d& v, double yaw, double pitch, double roll)
{
    cv::Matx33d ym, pm, rm;
    createYawPitchRollMatrices( yaw, pitch, roll, ym, pm, rm);
    return ym * rm * pm * v;
}   // end applyYawRollPitch



vector<cv::Mat_<float> > RFeatures::getCIELabChannels( const cv::Mat_<cv::Vec3b>& colImg)
{
    cv::Mat_<cv::Vec3f> cnvImg;
    colImg.convertTo( cnvImg, CV_32F, 1.0/255); // Make correct range of values across colour channels
    // Convert to CIE Lab colour space and take first channel
    cv::Mat labImg;
    cv::cvtColor( cnvImg, labImg, CV_BGR2Lab);
    // Take the first channel (the perceptual lightness)
    std::vector<cv::Mat_<float> > labChannels;
    cv::split( labImg, labChannels);

    double mn, mx;
    cv::minMaxLoc( labChannels[0], &mn, &mx);
    assert( mx <= 100);
    // labChannels[0] (L) in range [0,100]
    // labChannels[1] (a) in range [-127,127]
    // labChannels[2] (b) in range [-127,127]
    return labChannels;
}   // end getCIELabLightness



void RFeatures::convertToCIELab( cv::Mat_<cv::Vec3b>& img)
{
    cv::Mat labImg( img.size(), img.type());
    cv::cvtColor( img, labImg, CV_BGR2Lab);
    img = labImg;
}   // end convertToCIELab



cv::Mat RFeatures::getLightness( const cv::Mat_<cv::Vec3b>& img, double scale, int imgType)
{
    cv::Mat_<cv::Vec3f> f3img;
    img.convertTo( f3img, CV_32F, 1.0/255);  // Make correct range of values across colour channels
    cv::Mat labImg;
    cv::cvtColor( f3img, labImg, CV_BGR2Lab);
    std::vector<cv::Mat_<float> > labChannels;
    cv::split( labImg, labChannels);
    const cv::Mat_<float> lightMap = labChannels[0]; // 0 to 100
    cv::Mat outMap;
    lightMap.convertTo( outMap, imgType, scale/100);
    return outMap;
}   // end getLightness



cv::Mat_<byte> RFeatures::createDepthMask( const cv::Mat_<float> dimg)
{
    const cv::Rect rct(0,0,dimg.cols,dimg.rows);    // Whole image as passed in
    RFeatures::DepthSegmenter segmenter( dimg);
    segmenter.setMinMaxFromSubRect( rct);
    return segmenter.calcMostCommonDepthMask( rct);
}   // end createDepthMask



double RFeatures::pval( int depth, const byte* p)
{
    double v = 0;
    switch ( depth)
    {
        case CV_8U:
            v = *p;
            break;
        case CV_8S:
            v = double(*(char*)(p));
            break;
        case CV_16U:
            v = *(uint16_t*)(p);
            break;
        case CV_16S:
            v = double(*(int16_t*)(p));
            break;
        case CV_32S:
            v = double(*(int32_t*)(p));
            break;
        case CV_32F:
            v = *(float*)(p);
            break;
        case CV_64F:
            v = *(double*)(p);
            break;
        default:
            assert(false);
            break;
    }   // end switch
    return v;
}   // end pval



double getDiff( int depth, const byte* p1, const byte* p0)
{
    double diff = 0;
    switch ( depth)
    {
        case CV_8U:
            diff = *p1 - *p0;
            break;
        case CV_8S:
            diff = double(*(char*)(p1)) - *(char*)(p0);
            break;
        case CV_16U:
            diff = *(uint16_t*)(p1) - *(uint16_t*)(p0);
            break;
        case CV_16S:
            diff = double(*(int16_t*)(p1)) - *(int16_t*)(p0);
            break;
        case CV_32S:
            diff = double(*(int32_t*)(p1)) - *(int32_t*)(p0);
            break;
        case CV_32F:
            diff = *(float*)(p1) - *(float*)(p0);
            break;
        case CV_64F:
            diff = *(double*)(p1) - *(double*)(p0);
            break;
        default:
            assert(false);
            break;
    }   // end switch
    return diff;
}   // end getDiff



double RFeatures::calcHorizontalGrad( const cv::Mat &image, int row, int col, int ch)
{
    const int channels = image.channels();
    assert( ch >= 0 && ch < channels);
    double grad = 0;

    const int depth = image.depth();
    const int elemSz = (int)image.elemSize();    // Includes channel count
    const int elemSz1 = (int)image.elemSize1();

    if ( col > 0 && col < image.cols - 1)
    {
        const byte* p0 = &image.ptr(row)[(col-1)*elemSz + ch*elemSz1];
        const byte* p1 = &image.ptr(row)[(col+1)*elemSz + ch*elemSz1];
        grad = getDiff( depth, p1, p0);
    }   // end if
    else if ( col == 0) // At left edge
    {
        const byte* p0 = &image.ptr(row)[(col)*elemSz + ch*elemSz1];
        const byte* p1 = &image.ptr(row)[(col+1)*elemSz + ch*elemSz1];
        grad = getDiff( depth, p1, p0);
    }   // end else if
    else    // At right edge
    {
        const byte* p0 = &image.ptr(row)[(col-1)*elemSz + ch*elemSz1];
        const byte* p1 = &image.ptr(row)[(col)*elemSz + ch*elemSz1];
        grad = getDiff( depth, p1, p0);
    }   // end else

    return grad;
}   // end calcHorizontalGrad



double RFeatures::calcVerticalGrad( const cv::Mat &image, int row, int col, int ch)
{
    const int channels = image.channels();
    assert( ch >= 0 && ch < channels);
    double grad = 0;

    const int depth = image.depth();
    const int elemSz = (int)image.elemSize();    // Includes channel count
    const int elemSz1 = (int)image.elemSize1();
    const int c = col*elemSz + ch*elemSz1;

    if ( row > 0 && row < image.rows - 1)
    {
        const byte* p0 = &image.ptr(row-1)[c];
        const byte* p1 = &image.ptr(row+1)[c];
        grad = getDiff( depth, p1, p0);
    }   // end if
    else if ( row == 0) // At top row
    {
        const byte* p0 = &image.ptr(row)[c];
        const byte* p1 = &image.ptr(row+1)[c];
        grad = getDiff( depth, p1, p0);
    }   // end else if
    else    // At bottom row
    {
        const byte* p0 = &image.ptr(row-1)[c];
        const byte* p1 = &image.ptr(row)[c];
        grad = getDiff( depth, p1, p0);
    }   // end else

    return grad;
}   // end calcVerticalGrad


double RFeatures::l2sq( const cv::Vec3f& v)
{
    return v[0]*v[0] + v[1]*v[1] + v[2]*v[2];
}   // end l2sq
