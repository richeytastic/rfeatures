#include "SurfKeypoints.h"
using namespace RFeatures;


SurfKeypoints::SurfKeypoints( const cv::Mat &img, double t)
    : KeypointsDetector( img), threshold(t)
{
    working_image = img.clone();
}   // end ctor



Keypoints SurfKeypoints::find() const
{
    cv::SurfFeatureDetector fd( threshold);
    return KeypointsDetector::detectKeypoints( fd, working_image);
}   // end find



int SurfKeypoints::keypointDrawFlag() const
{
    return cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS;
}   // end keypointDrawFlag
