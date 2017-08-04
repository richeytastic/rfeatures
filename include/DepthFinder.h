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
#ifndef RFEATURES_DEPTH_FINDER_H
#define RFEATURES_DEPTH_FINDER_H

#include "FeatureUtils.h"
#include <boost/shared_ptr.hpp>


namespace RFeatures
{

class rFeatures_EXPORT DepthFinder
{
public:
    typedef boost::shared_ptr<DepthFinder> Ptr;

    static DepthFinder::Ptr create( const cv::Mat_<double> &depthIntImg, const cv::Mat_<int> &depthCountImg);

    // The prefered constructor takes an integral image of depth and an integral image
    // of points in the image having a valid depth value.
    DepthFinder( const cv::Mat_<double> &depthIntImg, const cv::Mat_<int> &depthCountImg);

    // This constructor loops through the point cloud to construct the integral image
    // of depth. Use the first constructor if depth integral image is already available.
    // The default focal vector for the provided point cloud points straight along
    // the positive z axis (into the scene).
    DepthFinder( const cv::Mat_<cv::Vec3f>& organisedPointCloud, const cv::Vec3d &focVec=cv::Vec3d(0,0,1));

    // Return the number of points within the provided rectangle having valid range values.
    int getValidRangeCount( const cv::Rect &rct) const;

    // Return the average depth over the provided rectangle. The provided rectangle
    // must not lie outside of the view image rectangle!
    double getAvgDepth( const cv::Rect &rct) const;

    // Get the depth at given indices. If no depth value exists at this point, 0 is returned.
    double getDepth( int row, int col) const;

    // The mean from that part of the base that sits within the view is returned.
    // Values larger than maxDepth are ignored.
    double getMeanDepthAlongBase( const cv::Rect &bb, float maxDepth) const;

    // Given a bounding box in the image, calculate it's actual size in metres with the assumption
    // that the extents of the given bounding box are all at more or less the same depth.
    // The depth of the bounding box is returned and the actual width/height of the box are
    // set in out parameter actSz.
    // ********* ASSUMES 90 DEGREE FOV **********
    double calcSize( const cv::Rect &boundBox, cv::Size2f &actSz) const;

    // Return the relative size of the given real model size at the given image position.
    // Some of the rectangle may be outside of the image bounds! If provided point is outside
    // of the image bounds, a rectangle with zero dimensions is returned. Parameter testSz is
    // used to define the size of the area over which the initial depth value is taken.
    // Setting this to a higher value will cause depth values to change more smoothly as
    // the pixel indices change. The returned rectangle is of correctly scaled dimensions
    // (according to measured depth) with its centre at x,y.
    cv::Rect getModelRect( const cv::Size2f &modelSize, int x, int y, int testSz=1) const;

    // More efficient version of getModelRect that sets the values of a provided modelRect.
    // Returns depth as used to create the modelRect.
    double calcModelRect( const cv::Size2f &modelSize, int x, int y, cv::Rect &modelRect, int testSz=1) const;

    // Calculate the pixel dimensions for a scale model bounding rectangle using the given
    // depth in metres and field of view in degrees. Only positive depth values are accepted.
    void calcScaleModel( const cv::Size2f &modelSize, int row, int col, double depth, double fov, cv::Rect &modelRect) const;

    // Set/get the height of the imaging camera above ground (in metres).
    static void setCameraHeight( double camHeight);
    static double getCameraHeight() { return camHeight;}

private:
    cv::Rect imgRct_;             // Convenience rectangle of image dimensions for calculating intersections
    cv::Mat_<double> depthIntImg;   // Integral image of depths
    cv::Mat_<int> depthCntImg;      // Valid range value counts

    static double camHeight;        // Height of camera from ground
    static const double MAX_DEPTH;  // Default distance when no depth value available
};  // end class DepthFinder



}   // end namespace

#endif
