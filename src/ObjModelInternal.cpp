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

#include <ObjModelInternal.h>
#include <boost/functional/hash.hpp>
#include <FeatureUtils.h>
using RFeatures::ObjMaterial;
using RFeatures::ObjPoly;
using RFeatures::ObjEdge;


/*********** ObjPoly *******************************/
ObjPoly::ObjPoly() {}

ObjPoly::ObjPoly( int v0, int v1, int v2)
{
    _fvindices[0] = v0;
    _fvindices[1] = v1;
    _fvindices[2] = v2;
}   // end ctor


// Two ObjPolys are the same if they have the same vertices
bool ObjPoly::operator==( const ObjPoly& p) const
{
    int v0 = _fvindices[0];
    int v1 = _fvindices[1];
    int v2 = _fvindices[2];
    reorderAscending( v0, v1, v2);

    int p0 = p._fvindices[0];
    int p1 = p._fvindices[1];
    int p2 = p._fvindices[2];
    reorderAscending( p0, p1, p2);

    return (v0 == p0) && (v1 == p1) && (v2 == p2);
}   // end operator==


bool ObjPoly::isAdjacent( const ObjPoly& p) const
{
    int v0 = _fvindices[0];
    int v1 = _fvindices[1];
    int v2 = _fvindices[2];
    reorderAscending( v0, v1, v2);

    int p0 = p._fvindices[0];
    int p1 = p._fvindices[1];
    int p2 = p._fvindices[2];
    reorderAscending( p0, p1, p2);

    const bool e0 = v0 == p0;
    const bool e1 = v1 == p1;
    const bool e2 = v2 == p2;
    return (e0 && e1) || (e1 && e2) || (e2 && e0);
}   // end isAdjacent


bool ObjPoly::opposite( int vid, int& other0, int& other1) const
{
    bool found = true;

    if ( vid == _fvindices[0])
    {
        other0 = _fvindices[1];
        other1 = _fvindices[2];
    }   // end if
    else if ( vid == _fvindices[1])
    {
        other0 = _fvindices[2];
        other1 = _fvindices[0];
    }   // end else if
    else if ( vid == _fvindices[2])
    {
        other0 = _fvindices[0];
        other1 = _fvindices[1];
    }   // end else if
    else
        found = false;

    return found;
}   // end opposite


int ObjPoly::opposite( int v0, int v1) const
{
    int vn = _fvindices[0];
    if ( vn == v0 || vn == v1)
    {
        vn = _fvindices[1];
        if ( vn == v0 || vn == v1)
            vn = _fvindices[2];
    }   // end if
    return vn;
}   // end opposite


int ObjPoly::index( int vidx) const
{
    int i = -1;
    if ( _fvindices[0] == vidx)
        i = 0;
    else if ( _fvindices[1] == vidx)
        i = 1;
    else if ( _fvindices[2] == vidx)
        i = 2;
    return i;
}   // end index


size_t RFeatures::HashObjPoly::operator()( const ObjPoly& u) const
{
    int v0 = u[0];
    int v1 = u[1];
    int v2 = u[2];
    reorderAscending( v0, v1, v2);   // Ensure v0 <= v1 <= v2
    size_t seed = 0;
    boost::hash_combine( seed, v0);
    boost::hash_combine( seed, v1);
    boost::hash_combine( seed, v2);
    return seed;
}   // end operator()


std::ostream& operator<<( std::ostream& os, const ObjPoly& f)
{
    os << f[0] << " " << f[1] << " " << f[2];
    return os;
}   // end operator<<


std::istream& operator>>( std::istream& is, ObjPoly& f)
{
    is >> f[0] >> f[1] >> f[2];
    return is;
}   // end operator>>
/*************************************************/


/*************** ObjEdge *****************************/
ObjEdge::ObjEdge() {}

ObjEdge::ObjEdge( int u0, int u1) : v0(u0), v1(u1)
{
    assert( v0 != v1);
    if ( v1 < v0)   // Ensure v0 is always smaller
        std::swap( v0, v1);
}   // end ctor

bool ObjEdge::operator==( const ObjEdge& e) const
{
    return e.v0 == v0 && e.v1 == v1;    // Requires vertices to be stored in ascending order
}   // end operator==

size_t RFeatures::HashObjEdge::operator()( const ObjEdge& u) const
{
    assert( u[0] < u[1]);
    size_t seed = 0;
    boost::hash_combine( seed, u[0]);
    boost::hash_combine( seed, u[1]);
    return seed;
}   // end operator()
/*************************************************/



/*************** ObjMaterial *****************************/
ObjMaterial::ObjMaterial() : _uvCounter(0) {}


// Update existing UV position
void ObjMaterial::updateUV( int uvID, const cv::Vec2f& newPos)
{
    assert( _uvIds.count(uvID) > 0);
    size_t key = hash( _uvs.at(uvID));
    _uv2id.erase(key);
    key = hash( newPos);
    _uv2id[key] = uvID;
    _uvs[uvID] = newPos;
}   // end updateUV


int ObjMaterial::_uvKey( const cv::Vec2f& uv)
{
    size_t k = hash( uv);
    if ( _uv2id.count(k) == 0)
    {
        _uvIds.insert(_uvCounter);
        _uvs[_uvCounter] = uv;
        _uv2id[k] = _uvCounter++;
    }   // end if
    return _uv2id.at(k);
}   // end _uvKey


void ObjMaterial::mapFaceUVs( int fid, const cv::Vec2f& uv0, const cv::Vec2f& uv1, const cv::Vec2f& uv2)
{
    _fids.insert(fid);

    const int i0 = _uvKey(uv0);
    const int i1 = _uvKey(uv1);
    const int i2 = _uvKey(uv2);

    _f2uv[fid][0] = i0;
    _f2uv[fid][1] = i1;
    _f2uv[fid][2] = i2;

    _uv2f[i0].insert(fid);
    _uv2f[i1].insert(fid);
    _uv2f[i2].insert(fid);
}   // end mapFaceUVs


// Erase this face ID from the material. If any of the UVs are
// no longer referenced by faces, remove them from the material.
void ObjMaterial::removeFaceUVs( int fid)
{
    const cv::Vec3i& uvis = _f2uv.at(fid);
    for ( int i = 0; i < 3; ++i)
    {
        int uvi = uvis[i];
        // Check for presence of _uv2f.at(uvi) because the entry may
        // have been deleted on a previous iteration through this loop.
        if ( _uv2f.count(uvi) > 0)
        {
            _uv2f.at(uvi).erase(fid);

            // If no more face references to this UV, we can remove the UV itself.
            if ( _uv2f.at(uvi).empty())
            {
                _uv2f.erase(uvi);
                size_t key = hash( _uvs.at(uvi));
                _uv2id.erase(key);
                _uvs.erase(uvi);
                _uvIds.erase(uvi);
            }   // end if
        }   // end if
    }   // end for

    _f2uv.erase(fid);
    _fids.erase(fid);
}   // end removeFaceUVs
/*********************************************************/


double RFeatures::roundndp( double x, size_t ndp)
{
    const double E = pow(10,ndp);
    const double y = double(long(x));
    const double z = double(long((x - y)*E + 0.5));
    return y + z/E;
}   // end roundndp


size_t RFeatures::hash( const cv::Vec3f& v, size_t ndp)
{
    size_t h = 0;
    boost::hash_combine( h, roundndp(v[0], ndp));
    boost::hash_combine( h, roundndp(v[1], ndp));
    boost::hash_combine( h, roundndp(v[2], ndp));
    return h;
}   // end hash


size_t RFeatures::hash( const cv::Vec2f& v, size_t ndp)
{
    size_t h = 0;
    boost::hash_combine( h, roundndp( v[0], ndp));
    boost::hash_combine( h, roundndp( v[1], ndp));
    return h;
}   // end hash

