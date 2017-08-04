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

#ifndef RFEATURES_VIEW_H
#define RFEATURES_VIEW_H

typedef unsigned char byte;
#include <boost/shared_ptr.hpp>
#include <opencv2/opencv.hpp>

#include "rFeatures_Export.h"
#include "FeatureExceptions.h"
#include <iostream>
using std::istream;
using std::ostream;


namespace RFeatures
{

struct rFeatures_EXPORT View
{
    typedef boost::shared_ptr<View> Ptr;

    explicit View( const cv::Size& sz);
    View( int width, int height);
    View( cv::Mat_<cv::Vec3b> img, cv::Mat_<float> rng);

    static View::Ptr create( const cv::Size& sz);
    static View::Ptr create( int width, int height);
    static View::Ptr create( cv::Mat_<cv::Vec3b> img, cv::Mat_<float> rng);

    cv::Vec3d posVec;   // Position of view in Euclidean space
    cv::Vec3d focalVec; // Focal vector orthogonal to camera plane into the scene
    cv::Vec3d upVec;    // Up vector

    cv::Mat_<cv::Vec3b> img2d;  // 2D image
    cv::Mat_<cv::Vec3f> points; // x,y,z original points corresponding to 2D pixel coord
    cv::Mat_<float> rngImg;     // Range image (simply the Z value from points)

    cv::Size size() const;
};  // end struct


rFeatures_EXPORT istream& operator>>( istream&, View::Ptr&);  // Read in a new View (NEW FORMAT)
rFeatures_EXPORT ostream& operator<<( ostream&, const View::Ptr&);    // Write out a View (NEW FORMAT)


// Convert the given image into a displayable range map with values closer to the image plane
// being brighter and dropping to black as distance increases. Range values outside of the given
// min and max ranges are ignored. If maxRng is left as -1, the full range of values in the range
// image will be used to create the grey scale values. Note that different images will have different
// maximum range values so the apparent depth scaling will not be comparable. For comparable apparent
// depth scaling across images, a set min and max range value should be used.
rFeatures_EXPORT cv::Mat_<byte> makeDisplayableRangeMap( const cv::Mat_<float>& rngImg,
                                                         float minRng=0, float maxRng=-1);



// Create maps of (absolute) change in both the horizontal (hmap) and vertical (vmap) directions.
// If mask is given, only calculate those values where mask != 0.
// img must be single channel of any depth.
rFeatures_EXPORT void createChangeMaps( const cv::Mat& img, cv::Mat& hmap, cv::Mat& vmap, bool useAbsolute=false, cv::Mat_<byte> mask=cv::Mat_<byte>());


#include "template/View_template.h"

}   // end namespace

#endif
