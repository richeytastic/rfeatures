#include "FastKeypoints.h"
#include "RFeatures.h"
using namespace RFeatures;


FastKeypoints::FastKeypoints( const cv::Mat &img, double t)
    : KeypointsDetector( img), threshold(t)
{
    working_image = img.clone();
}   // end ctor



Keypoints FastKeypoints::find() const
{
    cv::FastFeatureDetector fd( threshold);
    return KeypointsDetector::detectKeypoints( fd, working_image);
}   // end find
