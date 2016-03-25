#pragma once
#ifndef RFEATURES_SURF_KEYPOINTS_H
#define RFEATURES_SURF_KEYPOINTS_H

#include "KeypointsDetector.h"
using namespace RFeatures;


namespace RFeatures
{

class SurfKeypoints : public KeypointsDetector
{
public:
    SurfKeypoints( const cv::Mat &originalImage, double threshold);
    virtual ~SurfKeypoints(){}

    virtual Keypoints find() const;

protected:
    virtual int keypointDrawFlag() const;

private:
    cv::Mat working_image;
    double threshold;
};  // end class SurfKeypoints

}   // end namespace RFeatures

#endif

