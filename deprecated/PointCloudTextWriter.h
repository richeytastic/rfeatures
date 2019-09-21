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
 * Writes out a point cloud (or a portion of a point cloud if rectangle specified
 * for organised point clouds) as text.
 *
 * For organised point clouds:
 * Number of rows and columns stored are specified as:
 * rows cols
 * Thereafter, each point is written out on a new line in the format:
 * row col x y z r g b
 * where row and col are the indices of the point in the structured point cloud,
 * x y z are the floating point coordinates of the point, and
 * r g b are the byte valued colour channels.
 *
 * For unorganised point clouds:
 * Number of points as:
 * p
 * Thereafter, each point is written as:
 * x y z r g b
 * with the specification of these values as for the organised point cloud.
 *
 * Richard Palmer
 * November 2012
 */

#pragma once
#ifndef RFEATURES_POINT_CLOUD_TEXT_WRITER_H
#define RFEATURES_POINT_CLOUD_TEXT_WRITER_H

#include "PointCloudWriter.h"
#include "rFeatures_Export.h"
using RFeatures::PointCloudWriter;
#include <opencv2/opencv.hpp>


namespace RFeatures
{

class rFeatures_EXPORT PointCloudTextWriter : public PointCloudWriter
{
public:
    // Write out the area of the provided point cloud given by rect, or all of
    // the point cloud if rect not provided. Warning issued on stderr if
    // r is not NULL when provided point cloud is unorganised.
    PointCloudTextWriter( const PointCloud::Ptr&, const cv::Rect *r=0);
    virtual ~PointCloudTextWriter(){}

protected:
    virtual void write( ostream &os) const;

private:
    cv::Rect rect_;

    void writeOrganised( ostream&) const;
    void writeUnorganised( ostream&) const;
}; // end class

}  // end namespace

#endif
