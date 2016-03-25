#include "SiftKeypoints.h"
using namespace RFeatures;


SiftKeypoints::SiftKeypoints( const cv::Mat &img, double t, double et)
    : KeypointsDetector( img), threshold(t), edgeThreshold(et)
{
    working_image = img.clone();
}   // end ctor



Keypoints SiftKeypoints::find() const
{
    cv::SiftFeatureDetector fd( threshold, edgeThreshold);
    return KeypointsDetector::detectKeypoints( fd, working_image);
}   // end find



int SiftKeypoints::keypointDrawFlag() const
{
    return cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS;
}   // end keypointDrawFlag
