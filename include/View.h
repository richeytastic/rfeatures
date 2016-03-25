#pragma once
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


istream& operator>>( istream&, View::Ptr&);  // Read in a new View (NEW FORMAT)
ostream& operator<<( ostream&, const View::Ptr&);    // Write out a View (NEW FORMAT)


// Convert the given image into a displayable range map with values closer to the image plane
// being brighter and dropping to black as distance increases. Range values outside of the given
// min and max ranges are ignored. If maxRng is left as -1, the full range of values in the range
// image will be used to create the grey scale values. Note that different images will have different
// maximum range values so the apparent depth scaling will not be comparable. For comparable apparent
// depth scaling across images, a set min and max range value should be used.
cv::Mat_<byte> rFeatures_EXPORT makeDisplayableRangeMap( const cv::Mat_<float>& rngImg,
                                                         float minRng=0, float maxRng=-1);



// Create maps of (absolute) change in both the horizontal (hmap) and vertical (vmap) directions.
// If mask is given, only calculate those values where mask != 0.
// img must be single channel of any depth.
void createChangeMaps( const cv::Mat& img, cv::Mat& hmap, cv::Mat& vmap, bool useAbsolute=false, cv::Mat_<byte> mask=cv::Mat_<byte>());


#include "template/View_template.h"

}   // end namespace

#endif
