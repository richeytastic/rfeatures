#pragma once
#ifndef RFEATURES_ADAPTIVE_DEPTH_STRUCTURE_FILTER_H
#define RFEATURES_ADAPTIVE_DEPTH_STRUCTURE_FILTER_H

#include "AdaptiveDepthPatchScanner.h"
using RFeatures::PatchProcessor;
using RFeatures::AdaptiveDepthPatchScanner;
using RFeatures::PatchRanger;
#include "rFeatures_Export.h"


namespace RFeatures
{

class rFeatures_EXPORT AdaptiveDepthStructureFilter : private PatchProcessor
{
public:
    AdaptiveDepthStructureFilter( const cv::Mat_<float> depthMap, const cv::Size2f& realPatchSize);
    virtual ~AdaptiveDepthStructureFilter(){}

    cv::Mat_<byte> filter( float minRange=0, float maxRange=FLT_MAX);

private:
    const cv::Mat_<float> _rngImg;
    const PatchRanger _patchRanger;
    const cv::Size2f _rpatchSz;

    virtual void process( const cv::Point&, float, const cv::Rect&);

    cv::Mat_<byte> _outImg;
};  // end class

}   // end namespace

#endif
