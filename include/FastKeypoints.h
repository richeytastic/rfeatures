#pragma once
#ifndef RFEATURES_FAST_KEYPOINTS_H
#define RFEATURES_FAST_KEYPOINTS_H

#include "rFeatures_Export.h"
#include "KeypointsDetector.h"
using namespace RFeatures;


namespace RFeatures
{

class rFeatures_EXPORT FastKeypoints : public KeypointsDetector
{
public:
    FastKeypoints( const cv::Mat &originalImage, double threshold);
    virtual ~FastKeypoints(){}

    virtual Keypoints find() const;

private:
    cv::Mat working_image;
    double threshold;
};  // end class FastKeypoints

}   // end namespace RFeatures

#endif



