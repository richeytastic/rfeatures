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

#ifndef RFEATURES_OBJECT_MODEL_INTERNAL_H
#define RFEATURES_OBJECT_MODEL_INTERNAL_H

#include "rFeatures_Export.h"
#include <opencv2/opencv.hpp>
#include <unordered_map>
#include <unordered_set>
#include <iostream>
#include <cassert>
#include <memory>
#include <cmath>

#ifdef _WIN32
// Disable warnings about standard template library specialisations not being exported in the DLL interface
#pragma warning( disable : 4251)
#pragma warning( disable : 4275)
#endif

using IntSet = std::unordered_set<int>;

namespace RFeatures {

class ObjModel;

// A triangular face of the model with vertices stored in order of setting and defining the triangle's normal orientation.
struct rFeatures_EXPORT ObjPoly
{
    ObjPoly();
    ObjPoly( const ObjPoly&) = default;
    ObjPoly& operator=( const ObjPoly&) = default;
    ObjPoly( int v0, int v1, int v2);

    // Returns true if the parameter poly shares an edge with this one.
    bool isAdjacent( const ObjPoly&) const;
  
    // Get vertices that aren't v0. Returns false iff not found.
    bool opposite( int v0, int& v1, int& v2) const;

    // Returns the vertex that isn't v0 or v1 (or -1 if not found).
    int opposite( int v0, int v1) const;

    // Returns the index of vidx (0,1, or 2) as stored in this poly or -1 if not found.
    int index( int vidx) const;

    // ObjPoly are the same if they share the same vertices even if they're differently ordered.
    bool operator==( const ObjPoly&) const;

    inline const int* vertices() const { return _fvindices;}

    inline int at( int i) const { return _fvindices[i];}
    inline int operator[]( int i) const { return at(i);}
    inline int& operator[]( int i) { return _fvindices[i];}

private:
    // Vertex indices describing the triangle. Order defines normal orientation.
    int _fvindices[3];
    friend class ObjModel;
};  // end struct


// Define an edge as the end point vertex IDs
struct rFeatures_EXPORT ObjEdge
{
    ObjEdge();
    ObjEdge( const ObjEdge&) = default;
    ObjEdge& operator=( const ObjEdge&) = default;
    ObjEdge( int, int);    // Supply two different vertex IDs

    bool operator==( const ObjEdge& e) const;

    inline int at( int i) const { assert(i == 0 || i == 1); return i == 0 ? v0 : v1;}
    inline int operator[]( int i) const { return at(i);}
    inline int& operator[]( int i) { return i == 0 ? v0 : v1;}

private:
    int v0, v1; // Vertex indices (v0 always < v1 if via constructor)
};  // end struct


struct rFeatures_EXPORT ObjMaterial
{
    ObjMaterial();
    ObjMaterial( const ObjMaterial&) = default;
    ObjMaterial& operator=( const ObjMaterial&) = default;

private:
    int _uvCounter;
    cv::Mat _tx;
    IntSet _uvIds;
    std::unordered_map<int, cv::Vec2f> _uvs;    // UV IDs to UVs
    std::unordered_map<size_t, int>  _uv2id;    // Reverse mapping of UVs to IDs
    std::unordered_map<int, IntSet> _uv2f;      // UV IDs to polys
    std::unordered_map<int, cv::Vec3i> _f2uv;   // Poly to UV IDs in vertex matching order
    IntSet _fids;                               // Poly IDs that map to this material

    void updateUV( int uvId, const cv::Vec2f&);
    void mapFaceUVs( int fid, const cv::Vec2f&, const cv::Vec2f&, const cv::Vec2f&);
    int _uvKey( const cv::Vec2f&);
    void removeFaceUVs( int);

    friend class ObjModel;
};  // end struct


/**
 * Return x rounded to n decimal places.
 */
rFeatures_EXPORT double roundndp( double x, size_t n);

/**
 * Float values are rounded to ndp decimal places, then hash combined and returned.
 */
rFeatures_EXPORT size_t hash( const cv::Vec3f& v, size_t ndp=6);
rFeatures_EXPORT size_t hash( const cv::Vec2f& v, size_t ndp=6);

struct HashObjPoly : std::unary_function<ObjPoly, size_t> { size_t operator()( const ObjPoly&) const;};
struct HashObjEdge : std::unary_function<ObjEdge, size_t> { size_t operator()( const ObjEdge&) const;};


}   // end namespace

std::ostream& operator<<( std::ostream&, const RFeatures::ObjPoly&);
std::istream& operator>>( std::istream&, RFeatures::ObjPoly&);

#endif
