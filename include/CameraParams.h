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

#ifndef RFEATURES_CAMERA_PARAMS_H
#define RFEATURES_CAMERA_PARAMS_H

#ifdef _WIN32
#pragma warning( disable : 4251)
#endif

#include "rFeatures_Export.h"
#include <opencv2/opencv.hpp>
#include <iostream>

namespace RFeatures {

struct rFeatures_EXPORT CameraParams
{
    CameraParams(); // Camera's default position is at (0,0,1)
    CameraParams( const cv::Vec3f& pos,         
                  const cv::Vec3f& focus=cv::Vec3f(0,0,0),
                  const cv::Vec3f& up=cv::Vec3f(0,1,0),     // +ve Y
                  double fov=30);   // 30 degrees vertical field of view

    cv::Vec3f pos;      // Camera position
    cv::Vec3f focus;    // Camera focus
    cv::Vec3f up;       // Camera up vector
    double fov;         // Vertical field of view in degrees (must be > 0 and <= 180)

    // Reposition the camera r units from the focus (or a new focus if given)
    // along the line between the existing camera position and the focus.
    void setPositionFromFocus( double r, const cv::Vec3f* newFocus=NULL);

    // Project t's location onto the view plane as x,y proportional coordinates from the top left.
    cv::Point2f project( const cv::Vec3f& t) const;

    // Convenience function which takes the proportional coordinates and returns actual coords.
    cv::Point project( const cv::Vec3f& t, const cv::Size2i& viewPortSize) const;

    // Do perspective projection to return a world position with the X and Y values calculated and Z as given.
    cv::Vec3f project( const cv::Point2f& p, double Z) const;

    // Get the distance from t to the plane cooincident with the camera.
    double distance( const cv::Vec3f& t) const;

    // Set a new position based on a clockwise rotation along the up vector.
    void rotateAboutUpAxis( float degs);

    // Convenience calculations
    double fovRads() const; // fov * pi/180

    // Returns the standardised focal length between 0 and 1.
    double calcFocalLength() const; // cos(fovRads(fov)/2)/sin(fovRads(fov)/2)
};  // end struct

}   // end namespace


rFeatures_EXPORT std::ostream& operator<<( std::ostream& os, const RFeatures::CameraParams&);


#endif
