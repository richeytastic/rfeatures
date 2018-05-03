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

#ifndef RFEATURES_OBJ_MODEL_MOVER_H
#define RFEATURES_OBJ_MODEL_MOVER_H

#include "ObjModel.h"
#include "Orientation.h"

namespace RFeatures {

class rFeatures_EXPORT ObjModelMover
{
public:
    ObjModelMover();
    explicit ObjModelMover( const cv::Vec3d& translation);        // Create translation matrix only (no rotation).
    explicit ObjModelMover( const cv::Matx44d& transform);        // Set entire transformation matrix directly.

    // The following constructors allow for the optional incorporatation of a translatation AFTER the desired rotation.

    // Set rotation submatrix directly with (optional) subsequent translation.
    ObjModelMover( const cv::Matx33d& rotMat, const cv::Vec3d& t=cv::Vec3d(0,0,0));
    // Rotation from positive Z (normal) and positive Y (up) vectors with (optional) subsequent translation.
    ObjModelMover( const cv::Vec3d& posZ, const cv::Vec3d& posY, const cv::Vec3d& t=cv::Vec3d(0,0,0));
    // Rotation from angle and axis with (optional) subsequent translation.
    ObjModelMover( double radians, const cv::Vec3d& axis, const cv::Vec3d& t=cv::Vec3d(0,0,0));

    // Perform a transform prior to the existing transform.
    // Necessary for rotations where the object is not already at the origin.
    void prependTranslation( const cv::Vec3d&);

    // Returns the transformation matrix that is set in this ObjModelMover.
    inline const cv::Matx44d& transformMatrix() const { return _tmat;}

    void operator()( ObjModel::Ptr) const;  // Transform the provided object (adjust location of all of its vertices).

    // Transform a single vertex
    cv::Vec3f operator()( const cv::Vec3f&) const;
    cv::Vec3d operator()( const cv::Vec3d&) const;
    void operator()( cv::Vec3f&) const;   // In-place
    void operator()( cv::Vec3d&) const;   // In-place

    // Apply just the rotation submatrix (don't translate).
    cv::Vec3f rotate( const cv::Vec3f&) const;
    cv::Vec3d rotate( const cv::Vec3d&) const;
    void rotate( cv::Vec3f&) const;     // In-place rotation of given vertex
    void rotate( cv::Vec3d&) const;     // In-place rotation of given vertex

private:
    cv::Matx44d _tmat;  // Transformation matrix as homogeneous coordinates
};  // end class


// Given an orientation and position in space, construct and return a transformation matrix to reorient an
// object into standard position (position at (0,0,0), and orientation with normal vector as (0,1,0) and up
// vector as (0,0,1)) (translation applied first, followed by rotation).
rFeatures_EXPORT cv::Matx44d toStandardPosition( const Orientation&, const cv::Vec3f&);

}   // end namespace

#endif
