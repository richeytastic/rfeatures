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
#ifndef RFEATURES_POINT_CLOUD_TOOLS_H
#define RFEATURES_POINT_CLOUD_TOOLS_H

#include <string>
using std::string;
#include <vector>
using std::vector;
#include <opencv2/opencv.hpp>
#include "PointCloud.h"
using RFeatures::PointCloud;
#include "View.h"
using RFeatures::View;

#include "PointCloudOrienter.h"
#include "FeatureUtils.h"
//#include "ViewBuilder.h"


namespace RFeatures
{

class PointCloudTools
{
public:
    static rFeatures_EXPORT PointCloud::Ptr loadText( const string& fname);

    static rFeatures_EXPORT PointCloud::Ptr interpolate( const PointCloud::Ptr);

    // Takes a dense point cloud and creates front, left, rear and right faces
    // from a given camera position and vectors describing the camera orientation.
    // Returned array has four elements with the front, left, rear and right faces
    // in the respective index positions 0,1,2 and 3.
    // If too few points are found for a face, an empty vector is returned.
    // All faces must have some points.
    static rFeatures_EXPORT vector<PointCloud::Ptr> createFaces(
            const cv::Vec3d& frontVec, const cv::Vec3d& upVec, const cv::Vec3d& camPos,
            const PointCloud::Ptr, const cv::Vec3d scale=cv::Vec3d(1,1,1));


    // Creates a view from a point cloud and the specified view vectors.
    // Only works for organised point clouds.
    static rFeatures_EXPORT View::Ptr createView( const PointCloud::Ptr,
            const cv::Vec3d& pos, const cv::Vec3d& front, const cv::Vec3d& up);
};  // end class

}   // end namespace

#endif
