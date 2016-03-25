#include "StarKeypoints.h"
using namespace RFeatures;


StarKeypoints::StarKeypoints( const cv::Mat &img)
    : KeypointsDetector( img)
{
    working_image = img.clone();
}   // end ctor



Keypoints StarKeypoints::find() const
{
    cv::StarFeatureDetector fd;
    return KeypointsDetector::detectKeypoints( fd, working_image);
}   // end find
