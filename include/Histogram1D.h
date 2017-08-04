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
#ifndef RFEATURES_HISTOGRAM_1D_H
#define RFEATURES_HISTOGRAM_1D_H

#include "rFeatures_Export.h"
#include <opencv2/opencv.hpp>

namespace RFeatures
{

class rFeatures_EXPORT Histogram1D
{
public:
    Histogram1D();

    cv::MatND getHistogram( const cv::Mat &img);
    cv::Mat getHistogramImage( const cv::Mat &img);

    // Contrast stretch ignoring histogram bins with less than minVal pixels.
    static cv::Mat stretch( const cv::Mat &img, int minVal=0);

    static cv::Mat applyLookUp( const cv::Mat &img, const cv::Mat &lookup);

    static cv::Mat equalise( const cv::Mat &img);

private:
    int m_histSize[1]; // Number of bins
    float m_hranges[2];   // Min and max pixel values
    const float* m_ranges[1];
    int m_channels[1];  // Only 1 channel used here
};  // end class Histogram1D

}   // end namespace RFeatures

#endif
