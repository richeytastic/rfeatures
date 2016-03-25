#pragma once
#ifndef RFEATURES_FISH_2_RECT_H
#define RFEATURES_FISH_2_RECT_H

#include "rFeatures_Export.h"
#include <opencv2/opencv.hpp>

namespace RFeatures
{
class Fish2Rect
{
public:
    static rFeatures_EXPORT cv::Mat rectify( const cv::Mat& fishImg, double R, double A3, double A5);

    static rFeatures_EXPORT cv::Mat rectify( const cv::Mat& fishImg, double focalLengthPixels);
};  // end class
}   // end namespace

#endif
