#pragma once
#ifndef RFEATURES_EQUI_2_RECT_H
#define RFEATURES_EQUI_2_RECT_H

#include "rFeatures_Export.h"
#include <opencv2/opencv.hpp>

namespace RFeatures
{
class rFeatures_EXPORT Equi2Rect
{
public:
    // Give an equirectangular image and the focal length of the lens in pixels.
    Equi2Rect( const cv::Mat& equirectangular_image, float focalLen);

    // Calculate a 90 degree width, 90 degree height rectilinear face with the given
    // centre into the equirectangular image. Centre is given as a floating point value.
    cv::Mat calcRectilinear( const cv::Size& dims, float xcentre=0.5, float ycentre=0.5) const;

    // Calculate spherical lat,lon (radians) given an x,y floating point coordinate into a hemisphere half
    // of the spherical (360 degree) image where x and y can take on the values [-1,1].
    // Returned lat and lon are both in the range [-pi/4,pi/4].
    static void calc45LatLon( float x, float y, float centralLat, float centralLon, float& lat, float& lon);

private:
    const cv::Mat _eqimg;
    const float _focalLen;
};  // end class
}   // end namespace

#endif
