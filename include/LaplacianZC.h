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
#ifndef RFEATURES_LAPLACIAN_ZC_H
#define RFEATURES_LAPLACIAN_ZC_H

#include "rFeatures_Export.h"
#include <opencv2/opencv.hpp>

namespace RFeatures
{

class rFeatures_EXPORT LaplacianZC
{
public:
    LaplacianZC( int apertureSize);

    void setAperture( int apertureSize);

    // Compute the floating point laplacian matrix.
    cv::Mat computeLaplacian( const cv::Mat &image);

    // Get the laplacian result in an 8-bit image for display.
    cv::Mat getLaplacianImage( const cv::Mat &image);

    // Get a binary image of the zero-crossings. If the product of
    // two adjacent pixels is less than threshold, the zero-crossing will be ignored.
    cv::Mat getZeroCrossings( const cv::Mat &image, float threshold=1.0);

private:
    int aperture;   // Aperture size of laplacian kernel
};  // end class LaplacianZC

}   // end namespace RFeatures

#endif
