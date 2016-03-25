#pragma once
#ifndef RFEATURES_STAR_KEYPOINTS_H
#define RFEATURES_STAR_KEYPOINTS_H

#include "KeypointsDetector.h"
using namespace RFeatures;


namespace RFeatures
{

class rFeatures_EXPORT StarKeypoints : public KeypointsDetector
{
public:
    StarKeypoints( const cv::Mat &originalImage);
    virtual ~StarKeypoints(){}

    virtual Keypoints find() const;

private:
    cv::Mat working_image;
};  // end class StarKeypoints

}   // end namespace RFeatures

#endif



