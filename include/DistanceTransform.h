/************************************************************************
 * Copyright (C) 2017 Richard Palmer
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 ************************************************************************/

#ifndef RFEATURES_DISTANCE_TRANSFORM_H
#define RFEATURES_DISTANCE_TRANSFORM_H

#include "rFeatures_Export.h"
#include <opencv2/opencv.hpp>
#include <cassert>

namespace RFeatures {

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
void calcSignedRowDistances( const T* vals, float* distances, size_t N, T threshold, int power=1);

template <typename T>   // Unsigned version of above
void calcRowDistances( const T* vals, float* distances, size_t N, T threshold, int power=1);

template <typename T>   // Convenience function
cv::Mat_<float> calcSignedDistanceMap( const cv::Mat_<T>& m, T threshold, int power=1);

template <typename T>   // Convenience function
cv::Mat_<float> calcDistanceMap( const cv::Mat_<T>& m, T threshold, int power=1);


#include "template/DistanceTransform_template.cpp"

}   // end namespace

#endif
