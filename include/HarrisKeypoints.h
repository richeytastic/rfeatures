#pragma once
#ifndef RFEATURES_HARRIS_KEYPOINTS_H
#define RFEATURES_HARRIS_KEYPOINTS_H

#include "rFeatures_Export.h"
#include "KeypointsDetector.h"
using RFeatures::KeypointsDetector;


namespace RFeatures
{

class rFeatures_EXPORT HarrisKeypoints : public KeypointsDetector
{
public:
    HarrisKeypoints( const cv::Mat &originalImage, int maxCorners, double quality, int minDist);
    virtual ~HarrisKeypoints(){}

    virtual Keypoints find() const;

private:
    cv::Mat working_image;
    int maxCorners;
    double quality;
    int minDist;
};  // end class HarrisKeypoints

}   // end namespace RFeatures

#endif
