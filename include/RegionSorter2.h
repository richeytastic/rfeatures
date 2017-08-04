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
#ifndef RFEATURES_REGION_SORTER2_
#define RFEATURES_REGION_SORTER2_

#include <opencv2/opencv.hpp>
typedef unsigned int uint;


namespace RFeatures
{

class RegionSorter2
{
public:
    static RegionSorter2* create( uint dim);    // Square dimensions and only powers of 2

    // Add a rectangular region with given value. Returns the
    // maximum value so far.
    virtual double add( const cv::Rect& rct, double v) = 0;

    // Return the value and region of the highest valued area.
    virtual double findMax( cv::Rect& outRect) const = 0;
};  // end class

}   // end namespace

#endif
