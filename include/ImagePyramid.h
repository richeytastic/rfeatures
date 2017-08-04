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

#ifndef RFEATURES_IMAGE_PYRAMID_H
#define RFEATURES_IMAGE_PYRAMID_H

#ifdef _WIN32
#pragma warning( disable : 4251)
#endif

#include "rFeatures_Export.h"
#include <opencv2/opencv.hpp>
#include <vector>


namespace RFeatures
{

class rFeatures_EXPORT ImagePyramid
{
public:
    // The highest resolution image halucinates an image at twice the resolution of the
    // originally provided image. Parameter octaves specifies the number of whole halvings
    // of the image to be contained in the image pyramid. Parameter lambda controls the
    // number of levels between octaves (half resolution images) i.e. the number of images
    // you must go down in the image stack to find one at half the resolution.
    // Parameter minSize specifies the minimum allowed size for a dimension of an
    // image in the pyramid.
    // Leaving the default parameters gives an image pyramid with a single image
    // at twice the scale as the provided original.
    ImagePyramid( const cv::Mat &originalImage, int octaves=1, int lambda=1, int minSize=1);

    size_t size() const { return imgStack.size();}
    std::vector<cv::Mat> getImages() const { return imgStack;}
    std::vector<double> getScales() const { return scales;}

    // Return a single image in the pyramid. The highest resolution image is at
    // index 0 (which halucinates an image at twice the resolution of the
    // originally provided image). Returns an empty cv::Mat if idx is out of range.
    cv::Mat getImage( int idx) const;

    // Get the scale an image was produced at (relative to the original size of the image).
    // Returns <= 0 of provided index is invalid.
    double getScale( int idx) const;

private:
    std::vector<cv::Mat> imgStack;   // Highest resolution image at index 0 (twice power of original)
    std::vector<double> scales;
};  // end class

}   // end namespace

#endif
