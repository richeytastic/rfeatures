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

#ifndef RFEATURES_RANGE_MAP_ADJUSTER_H
#define RFEATURES_RANGE_MAP_ADJUSTER_H

#include "rFeatures_Export.h"
#include <opencv2/opencv.hpp>

namespace RFeatures {

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
