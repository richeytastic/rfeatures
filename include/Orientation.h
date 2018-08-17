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

#ifndef RFEATURES_ORIENTATION_H
#define RFEATURES_ORIENTATION_H

#ifdef _WIN32
#pragma warning( disable : 4251)
#endif

#include "rFeatures_Export.h"
#include <opencv2/opencv.hpp>
#include <boost/property_tree/ptree.hpp>
typedef boost::property_tree::ptree PTree;

namespace RFeatures {

class rFeatures_EXPORT Orientation
{
public:
    Orientation();                          // Norm=(0,0,1), Up=(0,1,0)
    explicit Orientation( const PTree&);    // Instantiate from a property tree
    Orientation( const cv::Vec3f& nvec, const cv::Vec3f& uvec);

    const cv::Vec3f& norm() const { return _nvec;}
    const cv::Vec3f& up() const { return _uvec;}

    cv::Vec3f& norm() { return _nvec;}
    cv::Vec3f& up() { return _uvec;}

    void rotate( const cv::Matx44d&);    // Only uses the rotation sub-matrix of the given 4x4 general transformation matrix

private:
    cv::Vec3f _nvec, _uvec;     // normal and up vector
};  // end class

rFeatures_EXPORT PTree& operator<<( PTree&, const Orientation&);        // Orientation writer to PTree
rFeatures_EXPORT const PTree& operator>>( const PTree&, Orientation&);  // Orientation reader from PTree

rFeatures_EXPORT void putVertex( PTree&, const cv::Vec3f&); // Creates keyed values "x", "y", and "z" in the provided record.

rFeatures_EXPORT void putNamedVertex( PTree&, const std::string&, const cv::Vec3f&); // Sets a child node with given label to the vertex.

rFeatures_EXPORT cv::Vec3f getVertex( const PTree&);        // Uses keyed values "x", "y", and "z" to create the returned vertex.

// Sets v to the named vertex if found in the given PTree and returns true if found (false if not).
rFeatures_EXPORT bool getNamedVertex( const PTree&, const std::string&, cv::Vec3f& v);

rFeatures_EXPORT std::ostream& operator<<( std::ostream&, const Orientation&);

}   // end namespace

#endif
