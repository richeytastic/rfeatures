#pragma once
#ifndef RFEATURES_RANGE_MAP_ADJUSTER_H
#define RFEATURES_RANGE_MAP_ADJUSTER_H

#include <opencv2/opencv.hpp>
#include "Superpixels.h"
using RFeatures::Superpixels;


namespace RFeatures
{

class rFeatures_EXPORT RangeMapAdjuster
{
public:
    RangeMapAdjuster( const cv::Mat_<float> &rngMap, float depthMax);

    // Use superpixel segmentation on the provided 2D image to segment the range
    // map based on the minimum range value within each superpixel.
    cv::Mat_<float> operator()( const cv::Mat_<cv::Vec3b> &img) const;

    // Inflate small areas of range according to a model which is assumed to sit
    // on the ground plane.
    cv::Mat_<float> operator()( const cv::Size2f &modelSize) const;

private:
    const cv::Mat_<float> rngMap_;
    const float depthMax_;

    cv::Mat_<float> adjustRangeMapHeight( const cv::Mat_<float>&, const cv::Size2f&) const;
};  // end class

}   // end namespace


#endif
