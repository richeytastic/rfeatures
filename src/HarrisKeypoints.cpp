#include "HarrisKeypoints.h"
using namespace RFeatures;


HarrisKeypoints::HarrisKeypoints( const cv::Mat &img, int maxC, double q, int minD)
    : KeypointsDetector( img)
{
    if ( img.channels() != 1)
        cv::cvtColor( img, working_image, CV_BGR2GRAY);
    else
        working_image = img.clone();

    maxCorners = maxC;
    if ( maxCorners < 1)
        maxCorners = 1;
    quality = q;
    minDist = minD;
    if ( minDist < 1)
        minDist = 1;
}   // end ctor



Keypoints HarrisKeypoints::find() const
{
    cv::GoodFeaturesToTrackDetector fd( maxCorners, quality, minDist);
    return KeypointsDetector::detectKeypoints( fd, working_image);
}   // end find
