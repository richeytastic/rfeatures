#pragma once
#ifndef RFEATURES_IMAGE_PROCESS_H
#define RFEATURES_IMAGE_PROCESS_H

#include <opencv2/opencv.hpp>
typedef unsigned char byte;
#include "rFeatures_Export.h"

namespace RFeatures
{

// Swap the left and right most end bytes of each pixel in the provided image.
// Returns image out for convenience.
rFeatures_EXPORT cv::Mat swapEndBytes( const cv::Mat &img, cv::Mat &out);

// Reduce the colour space of the image.
rFeatures_EXPORT void colourReduce( const cv::Mat &img, cv::Mat &out, int div=64);

// Sharpen the image by subtracting the Laplacian (without using explicit convolution kernel).
rFeatures_EXPORT void sharpen_OLD( const cv::Mat &img, cv::Mat &out);

// Sharpen the image by subtracting the Laplacian (using explicit convolution kernel).
rFeatures_EXPORT void sharpen( const cv::Mat &img, cv::Mat &out);


}  // end namespace RFeatures

#endif
