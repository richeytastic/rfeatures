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
