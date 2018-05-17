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
 * Create a two channel image where the first channel is the depth map of points from the
 * image plane and the second channel is the height of each point relative to the ground
 * plane (estimated by way of the provided up vector). The calculation for height
 * assumes a flat ground plane orthogonal to the provided up vector. Depth is calculated
 * from a plane orthogonal to the focal vector and incident with the origin (of the
 * point cloud coordinates).
 * Since the calculation for height assumes a flat ground plane, preciseness of the estimation
 * of the actual ground plane will decrease with distance from the origin, however this should
 * not radically affect the relative heights of objects of of changes in height of an object due
 * to its shape (since the curvature of the Earth does not typically deform the structure of an
 * object placed on it i.e. the ground plane is always locally flat).
 *
 * Richard Palmer
 * June 2012
 */

#pragma once
#ifndef RFEATURES_HEIGHT_DEPTH_IMAGER_H
#define RFEATURES_HEIGHT_DEPTH_IMAGER_H

#include "rFeatures_Export.h"
#include <cassert>
#include <exception>
#include <opencv2/opencv.hpp>
#include "PointCloud.h"
using RFeatures::PointCloud;
using RFeatures::PointCloudException;
#include "RangeImager.h"
using RFeatures::InvalidVectorException;


namespace RFeatures
{

class HeightDepthImager
{
public:
    // Returns CV_32FC2 image with actual depth in channel 0 and actual height in channel 1.
    // Image may be split into 2 separate channels to deal with the individual aspects.
 
    // Create a new HeightDepthImager instance from an organised point cloud (throws exception
    // if point cloud is not organised), and a focal vector into the scene (can be of any
    // length) and the scene up vector. The length of the up vector is used as the height
    // of the camera from the ground. Vectors of zero length will throw InvalidVectorExceptions.
    static rFeatures_EXPORT cv::Mat_<cv::Vec2f> makeRangeData( const PointCloud::Ptr pcloud, const cv::Vec3d& fv, const cv::Vec3d& uv)
                            throw (PointCloudException, InvalidVectorException);

    static rFeatures_EXPORT cv::Mat_<cv::Vec2f> makeRangeData( const cv::Mat_<cv::Vec3f>& points, const cv::Vec3d& fv, const cv::Vec3d& uv)
                            throw (InvalidVectorException);
};  // end class



/**
 * Class that uses the output range/height image from HeightDepthImager and a second
 * regular 2-D image of the same dimensions (but of any type and depth) to mask
 * the image according to provided height and depth ranges.
 *
 * Richard Palmer
 * June 2012
 */
class rFeatures_EXPORT ImageBounder
{
public:
    ImageBounder( const cv::Mat_<cv::Vec2f> &rmap, const cv::Mat &img);

    // Image returned will only contain pixels between the desired height
    // and depth ranges according to the distance map provided in c'tor.
    cv::Mat operator()( double minH, double maxH, double minD, double maxD) const;

private:
    const cv::Mat_<cv::Vec2f> rmap;
    const cv::Mat img;
};  // end class


}   // end namespace

#endif





