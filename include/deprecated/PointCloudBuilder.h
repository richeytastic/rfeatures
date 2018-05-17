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
 * Build a structured point cloud.
 * 
 * Richard Palmer
 * September 2012
 */

#pragma once
#ifndef RFEATURES_POINT_CLOUD_BUILDER
#define RFEATURES_POINT_CLOUD_BUILDER

#include <opencv2/opencv.hpp>
#include "PointDataBuilder.h"
using RFeatures::PointDataBuilder;
#include "PointCloud.h"
using RFeatures::PointCloud;
#include "rFeatures_Export.h"



namespace RFeatures
{

class rFeatures_EXPORT PointCloudBuilder : public PointDataBuilder
{
public:
    typedef boost::shared_ptr<PointCloudBuilder> Ptr;

    static PointCloudBuilder::Ptr create();

    virtual void reset( int width, int height);
    virtual void setPointPos( int row, int col, double x, double y, double z);
    virtual void setPointCol( int row, int col, byte r, byte g, byte b);

    inline PointCloud::Ptr getPointCloud() const { return m_pc;}

private:
    PointCloud::Ptr m_pc;
    
    int m_row1, m_row2;
    int m_col1, m_col2;
    double m_x, m_y, m_z;
    byte m_r, m_g, m_b;
    byte m_dataCount;
    void addPoint();

    PointCloudBuilder();
};  // end class

}   // end namespace

#endif
