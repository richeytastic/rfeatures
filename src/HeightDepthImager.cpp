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

#include "HeightDepthImager.h"
using RFeatures::HeightDepthImager;
#include <cmath>
#include <limits>


cv::Vec3d normFocalVector( const cv::Vec3d& f)
{
    double focLen = cv::norm(f);    // Focal length
    if ( focLen == 0.0)
        throw InvalidVectorException( "Zero vector cannot be used as focal vector!");
    return f / focLen;    // Normalise the focal vector
}   // end normFocalVector



void calcValue( cv::Vec2f& val, double x, double y, double z, const cv::Vec3d& fv, const cv::Vec3d& normUpVec, double camHeight)
{
    static const double INF = std::numeric_limits<double>::infinity();

    // Calculate depth on channel 0
    const cv::Vec3d p( x, y, z);
    double depth = fv.dot(p);    // Distance of p from projection plane (fv is normalised)
    if ( depth <= 0.0)
        depth = INF;    // Point either does not exist or is behind us

    val[0] = (float)depth;
    // Calculate height on channel 1
    const cv::Vec3d pp = p - depth*fv;    // Projection of p into image plane
    // Amount of pp in direction of normUpVec
    val[1] = (float)(camHeight + normUpVec.dot(pp));
}   // end calcValue



cv::Mat_<cv::Vec2f> HeightDepthImager::makeRangeData( const PointCloud::Ptr pc, const cv::Vec3d &f, const cv::Vec3d &uv)
                            throw (PointCloudException, InvalidVectorException)
{
    if ( !pc->isOrganised())
        throw PointCloudException( "Unorganised point clouds are not suitable for depth imaging!");
    if ( cv::norm(uv) == 0.0)   // Height of camera above ground
        throw InvalidVectorException( "Zero vector cannot be used as up vector!");
    const cv::Vec3d fv = normFocalVector(f);

    const int rows = (int)pc->getHeight();
    const int cols = (int)pc->getWidth();

    const double camHeight = cv::norm(uv);  // Height of camera
    const cv::Vec3d normUpVec = uv / camHeight; // Normalised up vector

    cv::Mat_<cv::Vec2f> rmap( cols, rows);    // Don't need to zero out
    for ( int i = 0; i < rows; ++i)
    {
        cv::Vec2f *rmapRow = rmap.ptr<cv::Vec2f>(i);    // Destination
        for ( int j = 0; j < cols; ++j)
        {
            // Calculate depth on channel 0
            const RFeatures::PointXYZRGB &pt = pc->at(i,j);
            calcValue( rmapRow[j], pt.x, pt.y, pt.z, fv, normUpVec, camHeight);
        }   // end for - cols
    }   // end for - rows

    return rmap;
}   // end makeRangeData



cv::Mat_<cv::Vec2f> HeightDepthImager::makeRangeData( const cv::Mat_<cv::Vec3f>& pc, const cv::Vec3d& f, const cv::Vec3d& uv)
    throw (InvalidVectorException)
{
    if ( cv::norm(uv) == 0.0)   // Height of camera above ground
        throw InvalidVectorException( "Zero vector cannot be used as up vector!");
    const cv::Vec3d fv = normFocalVector(f);

    const int rows = pc.rows;
    const int cols = pc.cols;

    const double camHeight = cv::norm(uv);  // Height of camera
    const cv::Vec3d normUpVec = uv / camHeight; // Normalised up vector

    cv::Mat_<cv::Vec2f> rmap( cols, rows);    // Don't need to zero out
    for ( int i = 0; i < rows; ++i)
    {
        cv::Vec2f *rmapRow = rmap.ptr<cv::Vec2f>(i);    // Destination
        for ( int j = 0; j < cols; ++j)
        {
            // Calculate depth on channel 0
            const cv::Vec3f& pt = pc(i,j);
            calcValue( rmapRow[j], pt[0], pt[1], pt[2], fv, normUpVec, camHeight);
        }   // end for - cols
    }   // end for - rows

    return rmap;
}   // end makeRangeData



using RFeatures::ImageBounder;
#include <cassert>
typedef unsigned char uchar;


    
ImageBounder::ImageBounder( const cv::Mat_<cv::Vec2f> &rmat, const cv::Mat &im)
    : rmap(rmat), img(im)
{
    assert( rmap.rows == img.rows && rmap.cols == img.cols);
}   // end ctor


cv::Mat ImageBounder::operator()( double minH, double maxH, double minD, double maxD) const
{
    const int channels = img.channels();
    const cv::Size sz = img.size();

    cv::Mat cimg( sz, img.type());

    for ( int i = 0; i < sz.height; ++i) // From top of image to bottom
    {
        const cv::Vec2f* rrow = rmap.ptr<cv::Vec2f>(i);
        const uchar* irow = img.ptr<uchar>(i);    // Source
        uchar* crow = cimg.ptr(i);    // Destination
        for ( int j = 0; j < sz.width; ++j)
        {
            // Height and depth values at this pixel
            const float &d = rrow[j][0];
            const float &h = rrow[j][1];

            const int pxi = j*channels;
            // In range?
            if ( (h >= minH) && (h <= maxH) && (d >= minD) && (d <= maxD))
            {
                for ( int k = 0; k < channels; ++k)
                    crow[pxi+k] = irow[pxi+k];
            }   // end if
            else
            {   // Pixel not in range so set black
                for ( int k = 0; k < channels; ++k)
                    crow[pxi+k] = 0;
            }   // end else
        }   // end for
    }   // end for

    return cimg;
}   // end operator()
