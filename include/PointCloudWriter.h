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
 * Abstract parent class for View writer types.
 *
 * Richard Palmer
 * September 2012
 */

#pragma once
#ifndef RFEATURES_POINT_CLOUD_WRITER
#define RFEATURES_POINT_CLOUD_WRITER

#include "PointDataWriter.h"
using RFeatures::PointDataWriter;
#include "PointCloud.h"
using RFeatures::PointCloud;


namespace RFeatures
{

class rFeatures_EXPORT PointCloudWriter : public PointDataWriter
{
public:
    virtual ~PointCloudWriter();

protected:
    virtual void write( ostream &os) const = 0;   // Implemented in device specific child classes

    explicit PointCloudWriter( const PointCloud::Ptr&); // No non-derived class construction
    const PointCloud::Ptr pcloud_;   // Object being written out
};  // end class PointCloudWriter


}   // end namespace

#endif
