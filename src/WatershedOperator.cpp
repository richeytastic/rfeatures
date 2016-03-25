#include "WatershedOperator.h"
using RFeatures::WatershedOperator;


WatershedOperator::WatershedOperator( const cv::Mat &img, const cv::Mat &mkrs)
{
    image = img.clone();
    mkrs.convertTo( markers, CV_32S);   // Convert to image of ints
}   // end ctor



cv::Mat WatershedOperator::findSegmentation() const
{
    cv::Mat mkrs = markers.clone();    // Image area markers (ints)
    cv::watershed( image, mkrs);
    return mkrs;
}   // end findSegmentation



// static
cv::Mat WatershedOperator::getSegmentedImage( const cv::Mat &mkrs)
{
    cv::Mat tmp;
    // All segments with labels higher than 255 will be assigned value 255
    mkrs.convertTo( tmp, CV_8U);
    return tmp;
}   // end getSegmentedImage



// static
cv::Mat WatershedOperator::getWatershedImage( const cv::Mat &mkrs)
{
    cv::Mat tmp;
    // Each pixel p is transformed into 255p+255 before conversion
    mkrs.convertTo( tmp, CV_8U, 255, 255);
    return tmp;
}   // end getWatershedImage
