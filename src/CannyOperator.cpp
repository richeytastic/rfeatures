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

#include "CannyOperator.h"
using RFeatures::CannyOperator;
#include <algorithm>
#include <cassert>


CannyOperator::CannyOperator( const cv::Mat &img, int lowT, int highT, const cv::Size fvDims)
    : FeatureOperator( img.size(), fvDims)
{
    assert( img.channels() == 1);
    // Set thresholds
    lowT = std::max<int>(0,lowT);
    highT = std::max<int>(0,highT);
    cv::Mat timg;
    cv::Canny( img, timg, lowT, highT);
    if ( timg.depth() != CV_32F)
        timg.convertTo( _cimg, CV_32F);
    else
        _cimg = timg;
}   // end ctor



void CannyOperator::getSamplingImages( vector<cv::Mat>& simgs) const
{
    simgs.push_back( _cimg);
}   // end simgs
