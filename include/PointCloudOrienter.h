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
 * Orient a point cloud in a different direction.
 * Richard Palmer
 * March 2013
 */

#pragma once
#ifndef RFEATURES_POINT_CLOUD_ORIENTER_H
#define RFEATURES_POINT_CLOUD_ORIENTER_H

#include <opencv2/opencv.hpp>
#include "PointCloud.h"
using RFeatures::PointCloud;
#include "rFeatures_Export.h"

namespace RFeatures
{

class rFeatures_EXPORT PointCloudOrienter
{
public:
    explicit PointCloudOrienter( const cv::Mat_<cv::Vec3f>& frontPointCloud);

    cv::Mat_<cv::Vec3f> copyToLeft() const;
    cv::Mat_<cv::Vec3f> copyToRear() const;
    cv::Mat_<cv::Vec3f> copyToRight() const;

private:
    const cv::Mat_<cv::Vec3f> _pc;
};  // end class

class rFeatures_EXPORT PointCloudOrienter2
{
public:
    explicit PointCloudOrienter2( const PointCloud::Ptr frontPointCloud);

    PointCloud::Ptr copyToLeft() const;
    PointCloud::Ptr copyToRear() const;
    PointCloud::Ptr copyToRight() const;

private:
    const PointCloud::Ptr _pc;
};  // end class

}   // end namespace

#endif
