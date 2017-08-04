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

/**
 * Compare images for similarity based on their histograms.
 */

#pragma once
#ifndef RLIB_IMAGE_COMPARATOR_H
#define RLIB_IMAGE_COMPARATOR_H

#include <ImageProcess.h>
#include <ColourHistogram.h>
using namespace RFeatures;
#include <opencv2/opencv.hpp>


namespace RFeatures
{

class ImageComparator
{
public:
    ImageComparator( int factor, const cv::Mat &reference_img);

    // Returns a number between 0 and 1 where numbers closer to
    // 1 are more "similar" to the reference image.
    double operator()( const cv::Mat &comparison_img);

private:
    cv::MatND refH;
    cv::MatND inputH;

    ColourHistogram hist;
    cv::Size imgsz;  // Size of the images (must be equal)
    int div;

    void setColourReduction( int factor);
    void setReferenceImage( const cv::Mat &img);
};  // end class ImageComparator

}   // end namespace RFeatures

#endif
