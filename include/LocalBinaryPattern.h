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
#ifndef RFEATURES_LOCAL_BINARY_PATTERN_H
#define RFEATURES_LOCAL_BINARY_PATTERN_H

#include "FeatureUtils.h"
#include "FeatureOperator.h"


namespace RFeatures
{

class rFeatures_EXPORT LocalBinaryPattern : public RFeatures::FeatureOperator
{
public:
    // img: must be single channel - can be any depth.
    explicit LocalBinaryPattern( const cv::Mat& img);
    virtual ~LocalBinaryPattern();

    // Extracts the local binary pattern over the given rectangle for
    // three different scales given by the original dimensions of rct,
    // 1.5 times the dimensions of rct, and twice the dimensions of rct.
    // If not already divisible by three when rct is passed in, the original
    // dimensions of rct are first set to the smallest integers divisible
    // by three. For each of the three different scales, the 8-bit binary
    // patterns produced are packed into a cv::Vec3b for return.
    cv::Vec3b extractBinaryPattern( const cv::Rect& rct) const;

    // Extracts the bit pattern over the given rectangle.
    byte calcBitPattern( const cv::Rect&) const;

    // Map img to a returned image of the same dimensions of
    // type CV_8UC(dims.size()). cv::Rect instances of the corresponding
    // dimensions are used at each pixel position to compile the elements
    // of the output map. This function allows for arbitrary pixel dimensions.
    static cv::Mat map( const cv::Mat& img, const std::vector<size_t>& dims);

protected:
    // Wraps extractBinaryPatterns above.
    virtual cv::Mat_<float> extract( const cv::Rect& rct) const;

private:
    const cv::Rect _imgRct;
    const cv::Mat_<float> _intImg;
};   // end class

}   // end namespace


#endif
