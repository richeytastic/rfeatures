#pragma once
#ifndef RFEATURES_DISTANCE_TRANSFORM_H
#define RFEATURES_DISTANCE_TRANSFORM_H

#include "rFeatures_Export.h"
#include <opencv2/opencv.hpp>
#include <cassert>

namespace RFeatures
{

struct DistanceTransform
{
// img values == 0 are treated as foreground points.
// Returns the squared Euclidean distance transform.
static rFeatures_EXPORT cv::Mat_<int> calcdt( const cv::Mat& img)
{
    assert( img.channels() == 1);
    cv::Mat_<int> im;
    img.convertTo( im, CV_32S);
    inplace( im);
    return im;
}   // end calcdt

static rFeatures_EXPORT void inplace( cv::Mat_<int>& im);    // Calculate distance transform in-place
};  // end struct


// Sets array D with pixel distances to target pixels (> thresh) along rows.
// Euclidean distances can be set using power=2. For all powers, pixels to the
// right of target pixels with be signed positive and pixels to the left of target
// pixels will be signed negatively. For pixels having target pixels of equal distance
// to the left and right, the value will be positive.
// The element lengths of vals and distances must match and be equal to N.
template <typename T>
rFeatures_EXPORT void calcSignedRowDistances( const T* vals, float* distances, size_t N, T threshold, int power=1);

template <typename T>   // Unsigned version of above
rFeatures_EXPORT void calcRowDistances( const T* vals, float* distances, size_t N, T threshold, int power=1);

template <typename T>   // Convenience function
rFeatures_EXPORT cv::Mat_<float> calcSignedDistanceMap( const cv::Mat_<T>& m, T threshold, int power=1);

template <typename T>   // Convenience function
rFeatures_EXPORT cv::Mat_<float> calcDistanceMap( const cv::Mat_<T>& m, T threshold, int power=1);


#include "template/DistanceTransform_template.h"

}   // end namespace

#endif
