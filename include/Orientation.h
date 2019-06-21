/************************************************************************
 * Copyright (C) 2019 Richard Palmer
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

#include "FeatureUtils.h"
#include <boost/property_tree/ptree.hpp>
typedef boost::property_tree::ptree PTree;

namespace RFeatures {

class rFeatures_EXPORT Orientation
{
public:
    /**
     * Create a default orientation object with normal +Z (i.e. [0,0,1]) and up vector as +Y (i.e. [0,1,0]).
     */
    Orientation();

    /**
     * Instantiate from a property tree.
     */
    explicit Orientation( const PTree&);
    Orientation( const cv::Vec3f& nvec, const cv::Vec3f& uvec);

    /**
     * Get the normal vector as the normalised first 3 elements as the 3rd column,
     * and the up vector as the normalised first 3 elements of the 2nd column.
     */
    Orientation( const cv::Matx44d&);

    /**
     * Set the normal and up vectors.
     */
    void setN( const cv::Vec3f& n) { cv::normalize(n, _nvec);}
    void setU( const cv::Vec3f& u) { cv::normalize(u, _uvec);}

    const cv::Vec3f& nvec() const { return _nvec;}
    const cv::Vec3f& uvec() const { return _uvec;}

    /**
     * Rotate this orientation by the given rotation matrix.
     */
    void rotate( const cv::Matx33d&);

    /**
     * Rotate only using the rotation sub-matrix of the given 4x4 general transformation matrix.
     */
    void rotate( const cv::Matx44d&);

    /**
     * Return this orientation object as a homogeneous matrix with optional translation.
     */
    cv::Matx44d asMatrix( const cv::Vec3d& t=cv::Vec3d(0,0,0)) const;

    bool operator==( const Orientation&) const;
    bool operator!=( const Orientation& o) const { return !operator==(o);}

private:
    cv::Vec3f _nvec, _uvec;     // normal and up vector
};  // end class

rFeatures_EXPORT PTree& operator<<( PTree&, const Orientation&);        // Orientation writer to PTree
rFeatures_EXPORT const PTree& operator>>( const PTree&, Orientation&);  // Orientation reader from PTree

rFeatures_EXPORT void putVertex( PTree&, const cv::Vec3f&); // Creates keyed values "x", "y", and "z" in the provided record.

rFeatures_EXPORT void putNamedVertex( PTree&, const std::string&, const cv::Vec3f&); // Sets child node with given label to vertex.

rFeatures_EXPORT cv::Vec3f getVertex( const PTree&);        // Uses keyed values "x", "y", and "z" to create the returned vertex.

// Sets v to the named vertex if found in the given PTree and returns true if found (false if not).
rFeatures_EXPORT bool getNamedVertex( const PTree&, const std::string&, cv::Vec3f& v);

rFeatures_EXPORT std::ostream& operator<<( std::ostream&, const Orientation&);

}   // end namespace

#endif
