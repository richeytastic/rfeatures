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
 * Pure virtual parent of all objects that want to create
 * data structures from structured point cloud data.
 *
 * Richard Palmer
 * September 2012
 */

#pragma once
#ifndef RFEATURES_POINT_DATA_BUILDER_H
#define RFEATURES_POINT_DATA_BUILDER_H

#include <cstdlib>
#include <boost/shared_ptr.hpp>
typedef unsigned char byte;


namespace RFeatures
{

class PointDataBuilder
{
public:
    typedef boost::shared_ptr<PointDataBuilder> Ptr;

    virtual void setPointPos( int row, int col, double x, double y, double z){}
    virtual void setPointRange( int row, int col, double rng){}
    virtual void setPointCol( int row, int col, byte r, byte g, byte b){}

    virtual void reset( int width, int height) = 0;
};  // end class


}   // end RFeatures

#endif
