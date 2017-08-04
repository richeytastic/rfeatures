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

#ifndef RFEATURES_SOBEL_MAKER_H
#define RFEATURES_SOBEL_MAKER_H

#ifdef _WIN32
#pragma warning( disable : 4251)
#endif

#include "rFeatures_Export.h"
#include <opencv2/opencv.hpp>

namespace RFeatures
{

class rFeatures_EXPORT SobelMaker
{
public:
    explicit SobelMaker( const cv::Mat& img);   // img must be single channel of any depth

    // Returned maps have values in range [0,1]
    cv::Mat_<float> makeSobelD1( int ksize=3) const;
    cv::Mat_<float> makeSobelD2( int ksize=3) const;

private:
    cv::Mat_<float> _img;
};  // end class


}   // end namespace

#endif
