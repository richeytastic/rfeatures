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
#ifndef RFEATURES_WATERSHED_OPERATOR_H
#define RFEATURES_WATERSHED_OPERATOR_H

#include <opencv2/opencv.hpp>
#include "rFeatures_Export.h"

namespace RFeatures
{

class rFeatures_EXPORT WatershedOperator
{
public:
    // Image can be a regular 3 channel image. Markers should be a single channel
    // image with white (255) pixels denoting foreground objects, grey (128) pixels
    // denoting background and black (0) denoting unknown image areas.
    WatershedOperator( const cv::Mat &img, const cv::Mat &markers);
    virtual ~WatershedOperator(){}

    // Return the matrix of ints denoting the differently segmented pixels.
    // May be passed to getSegmentedImage or getWatershedImage to produce
    // a grey level image to display.
    cv::Mat findSegmentation() const;

    // Returns the segmented image for display.
    // Foreground image areas are white (255),
    // Background image areas are grey (128),
    // Unknown image areas are black (0).
    static cv::Mat getSegmentedImage( const cv::Mat &markers);

    // Returns the watershed boundaries as white on black background.
    static cv::Mat getWatershedImage( const cv::Mat &markers);

private:
    cv::Mat image;  // Image to process
    cv::Mat markers;    // Image markers (ints)
};  // end class WatershedOperator

}   // end namespace RFeatures

#endif
