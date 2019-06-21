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

#ifndef RFEATURES_TRANSFORMER_H
#define RFEATURES_TRANSFORMER_H

#ifdef _WIN32
// Disable warning about cv::Matx44d not being exported in DLL interface.
#pragma warning( disable : 4251)
#endif

#include "rFeatures_Export.h"
#include <opencv2/opencv.hpp>

namespace RFeatures {

class rFeatures_EXPORT Transformer
{
public:
    Transformer();                                // Identity matrix
    Transformer( const cv::Vec3f& translation);   // Create translation matrix only (no rotation).
    Transformer( const cv::Vec3d& translation);   // Create translation matrix only (no rotation).
    Transformer( const cv::Matx44d& transform);   // Set entire transformation matrix directly.

    // The following constructors allow for the optional incorporatation of a translatation AFTER the desired rotation.

    // Set rotation submatrix directly with (optional) subsequent translation.
    Transformer( const cv::Matx33d& rotMat, const cv::Vec3d& t=cv::Vec3d(0,0,0));
    // Rotation from positive Z (normal) and positive Y (up) vectors with (optional) subsequent translation.
    Transformer( const cv::Vec3d& posZ, const cv::Vec3d& posY, const cv::Vec3d& t=cv::Vec3d(0,0,0));
    // Rotation with angle about given axis with (optional) subsequent translation.
    Transformer( double radians, const cv::Vec3d& axis, const cv::Vec3d& t=cv::Vec3d(0,0,0));
    Transformer( double radians, const cv::Vec3f& axis, const cv::Vec3f& t=cv::Vec3f(0,0,0));

    // Perform a transform prior to the existing transform.
    // Necessary for rotations where the object is not already at the origin.
    void prependTranslation( const cv::Vec3d&);

    // Append parameter's matrix by this one and set in this one (returning self).
    // The parameter matrix will be applied AFTER the matrix in this mover.
    Transformer& operator*( const Transformer&);

    inline const cv::Matx44d& matrix() const { return _tmat;}   // Returns Transformer's transformation matrix.
    inline const cv::Matx44d& operator()() const { return _tmat;} // Synonym

    cv::Vec3f transform( const cv::Vec3f&) const;   // Return transformed
    cv::Vec3d transform( const cv::Vec3d&) const;   // Return transformed
    void transform( cv::Vec3f&) const;   // In-place transform
    void transform( cv::Vec3d&) const;   // In-place transform

    // Transform the provided object (adjust all of its vertices).
    //void transform( ObjModel::Ptr) const;

    // Apply just the rotation submatrix (don't translate).
    cv::Vec3f rotate( const cv::Vec3f&) const;
    cv::Vec3d rotate( const cv::Vec3d&) const;
    void rotate( cv::Vec3f&) const;     // In-place rotation of given vertex
    void rotate( cv::Vec3d&) const;     // In-place rotation of given vertex

private:
    cv::Matx44d _tmat;  // Transformation matrix as homogeneous coordinates
};  // end class

}   // end namespace

#endif
