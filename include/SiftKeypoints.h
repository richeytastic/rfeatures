#pragma once
#ifndef RFEATURES_SIFT_KEYPOINTS_H
#define RFEATURES_SIFT_KEYPOINTS_H

#include "KeypointsDetector.h"
using namespace RFeatures;


namespace RFeatures
{

class SiftKeypoints : public KeypointsDetector
{
public:
    SiftKeypoints( const cv::Mat &originalImage,
            double threshold,
            double edgeThreshold);

    virtual ~SiftKeypoints(){}

    virtual Keypoints find() const;

protected:
    virtual int keypointDrawFlag() const;

private:
    cv::Mat working_image;
    double threshold;
    double edgeThreshold;
};  // end class SiftKeypoints

}   // end namespace RFeatures

#endif


