#pragma once
#ifndef RFEATURES_ADAPTIVE_DEPTH_MIN_FILTER_H
#define RFEATURES_ADAPTIVE_DEPTH_MIN_FILTER_H

#include "rFeatures_Export.h"
#include "AdaptiveDepthPatchScanner.h"

namespace RFeatures
{

class rFeatures_EXPORT AdaptiveDepthMinFilter
{
public:
    AdaptiveDepthMinFilter( const cv::Mat_<float> depthMap, const cv::Size2f& realPatchSize);

    cv::Mat_<float> filter( float minRange=0, float maxRange=FLT_MAX);

private:
    const cv::Mat_<float> _rngImg;
    const cv::Size2f _rpatchSz;
};  // end class

}   // end namespace

#endif
