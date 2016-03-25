#pragma once
#ifndef RFEATURES_DEPTH_DIFF_H
#define RFEATURES_DEPTH_DIFF_H

#include "FeatureOperator.h"

namespace RFeatures
{

// Four different types of patch point distributions
enum PatchPointType
{
    FOUR_PT,    // 4*3/2 = 6
    FIVE_PT,    // 5*4/2 = 10
    NINE_PT,    // 9*8/2 = 36
    THIRTEEN_PT // 13*12/2 = 78
};  // end enum


int getPointPatchLength( const PatchPointType ddt);



class rFeatures_EXPORT DepthDiff : public RFeatures::FeatureOperator
{
public:
    DepthDiff( const cv::Mat_<float>& img, PatchPointType, float sensitivity=1.0);
    virtual ~DepthDiff(){}

protected:
    virtual cv::Mat_<float> extract( const cv::Rect&) const;

private:
    const cv::Mat_<float> _img;
    const PatchPointType _ppt;
    const float _sensitivity;
};  // end class

}   // end namespace

#endif

