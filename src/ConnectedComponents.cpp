#include "ConnectedComponents.h"
using namespace RFeatures;


ConnectedComponents::ConnectedComponents( const cv::Mat &img, int min, int max)
{
    if ( max < 0)
        max = (img.rows * img.cols * img.rows);
    image = img.clone();
    setContourLengths( min, max);
}   // end ctor



void ConnectedComponents::setContourLengths( int minL, int maxL)
{
    if ( minL < 1)
        minL = 1;
    if ( maxL < 1)
        maxL = 1;
    if ( maxL <= minL)
        maxL = minL + 1;
    minLen = (size_t)minL;
    maxLen = (size_t)maxL;
}   // end setContourLengths



ContoursVector ConnectedComponents::findComponents() const
{
    ContoursVector components;
    cv::findContours( image, components, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
    return filterComponents( components);
}   // end findComponents



ContoursVector ConnectedComponents::filterComponents( const ContoursVector &cvs) const
{
    ContoursVector newComps;

    ContoursVector::const_iterator itc = cvs.begin();
    while (itc != cvs.end())
    {
        if (( itc->size() >= minLen) && (itc->size() < maxLen))
            newComps.push_back( *itc);
        ++itc;
    }   // end while

    return newComps;
}   // end filterComponents



// static
void ConnectedComponents::drawContours( const ContoursVector &cvs, cv::Mat &img, cv::Scalar col, int thick)
{
    cv::drawContours( img, cvs,
            -1,     // Draw all contours
            col, thick);
}   // end drawContours



void ConnectedComponents::drawConvexHulls( const ContoursVector &cvs, cv::Mat &img, cv::Scalar col, int thick)
{
    ContoursVector hulls;
    ContoursVector::const_iterator itc = cvs.begin();
    while (itc != cvs.end())
    {
        vector<cv::Point> hull;
        cv::convexHull( cv::Mat(*itc), hull);   // Create convex hull from contour
        hulls.push_back(hull);
        //drawContour( hull, img, col, thick);
        ++itc;
    }   // end while

    drawContours( hulls, img, col, thick);
}   // end drawConvexHulls



void ConnectedComponents::drawApproxPolys( const ContoursVector &cvs, cv::Mat &img, cv::Scalar col, int thick)
{
    ContoursVector polys;
    ContoursVector::const_iterator itc = cvs.begin();
    while (itc != cvs.end())
    {
        vector<cv::Point> poly;
        // Create approximate polygon from contour.
        // 5 gives the accuracy of the polygon and true denotes a closed shape.
        cv::approxPolyDP( cv::Mat(*itc), poly, 7, true);
        polys.push_back(poly);
        //drawContour( hull, img, col, thick);
        ++itc;
    }   // end while

    drawContours( polys, img, col, thick);
}   // end drawApproxPolys
