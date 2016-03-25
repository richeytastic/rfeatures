/**
 * Threshold the edges of a depth map.
 * Looks at the minimum, maximum and central values of a scaling patch
 * and calculates the 3 differences squared which are then tested against
 * the provided threshold. Values larger than the threshold are set to
 * white in the returned map.
 */

#pragma once
#ifndef RFEATURES_ADAPTIVE_DEPTH_SEGMENTER_H
#define RFEATURES_ADAPTIVE_DEPTH_SEGMENTER_H

#include "AdaptiveDepthPatchScanner.h"
using RFeatures::PatchProcessor;
using RFeatures::AdaptiveDepthPatchScanner;
#include "rFeatures_Export.h"


namespace RFeatures
{

class rFeatures_EXPORT AdaptiveDepthSegmenter : private PatchProcessor
{
public:
    AdaptiveDepthSegmenter( const cv::Mat_<float> depthMap, const cv::Size2f& realPatchSize, float threshVal=5);
    virtual ~AdaptiveDepthSegmenter(){}

    cv::Mat_<byte> filter( float minRange=0, float maxRange=FLT_MAX);

private:
    const cv::Mat_<float> _rngImg;
    const cv::Size2f _rpatchSz;
    const float _threshVal;

    virtual void process( const cv::Point&, float, const cv::Rect&);

    cv::Mat_<byte> _outImg;
};  // end class

}   // end namespace

#endif
