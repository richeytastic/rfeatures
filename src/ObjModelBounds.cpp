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

#include <ObjModelBounds.h>
#include <FeatureUtils.h>
using RFeatures::ObjModelBounds;
using RFeatures::ObjModel;
using RFeatures::ObjPoly;
#include <algorithm>
#include <iomanip>
#include <cassert>

namespace {

void expandBounds( const ObjModel& model, const cv::Vec3f& newv, cv::Vec6d& bounds)
{
    if ( newv[0] < bounds[0])
        bounds[0] = newv[0];
    if ( newv[0] > bounds[1])
        bounds[1] = newv[0];

    if ( newv[1] < bounds[2])
        bounds[2] = newv[1];
    if ( newv[1] > bounds[3])
        bounds[3] = newv[1];

    if ( newv[2] < bounds[4])
        bounds[4] = newv[2];
    if ( newv[2] > bounds[5])
        bounds[5] = newv[2];
}   // end expandBounds


cv::Vec6d findVertexBounds( const ObjModel& model, const IntSet& vidxs)
{
    cv::Vec6d bounds;
    bounds[0] = DBL_MAX;    // Xmin
    bounds[1] = -DBL_MAX;   // Xmax
    bounds[2] = DBL_MAX;    // Ymin
    bounds[3] = -DBL_MAX;   // Ymax
    bounds[4] = DBL_MAX;    // Zmin
    bounds[5] = -DBL_MAX;   // Zmax
    std::for_each( std::begin(vidxs), std::end(vidxs), [&](int v){ expandBounds( model, model.vtx(v), bounds);});
    return bounds;
}   // end findVertexBounds

}   // end namespace


ObjModelBounds::Ptr ObjModelBounds::create( const ObjModel& model, const IntSet* vset)
{
    return Ptr( new ObjModelBounds( model, vset));
}   // end create


ObjModelBounds::Ptr ObjModelBounds::create( const ObjModel& model, const IntSet& pset)
{
    return Ptr( new ObjModelBounds( model, pset));
}   // end create


void ObjModelBounds::_init( const ObjModel& model, const IntSet& vset)
{
    _vbnd = findVertexBounds( model, vset);
    _xlen = fabs(_vbnd[0] - _vbnd[1]);
    _ylen = fabs(_vbnd[2] - _vbnd[3]);
    _zlen = fabs(_vbnd[4] - _vbnd[5]);
    cv::Vec3f c0, c1;
    corners( c0, c1);
    _diag = ObjModelBounds::diagonal( c0, c1);
}   // end _init


ObjModelBounds::ObjModelBounds( const ObjModel& model, const IntSet* vset)
{
    if ( !vset)
        vset = &model.vtxIds();
    _init( model, *vset);
}   // end ctor


ObjModelBounds::ObjModelBounds( const ObjModel& model, const IntSet& pset)
{
    IntSet vset;
    for ( int fid : pset)
    {
        const ObjPoly& p = model.face(fid);
        vset.insert(p[0]);
        vset.insert(p[1]);
        vset.insert(p[2]);
    }   // end for
    _init( model, vset);
}   // end ctor


void ObjModelBounds::transform( const cv::Matx44d& tmat)
{
    cv::Vec3f c0, c1;
    corners( c0, c1);
    RFeatures::transform( tmat, c0);
    RFeatures::transform( tmat, c1);
    _vbnd = ObjModelBounds::as6d( c0, c1);
}   // end transform


void ObjModelBounds::corners( cv::Vec3f& minc, cv::Vec3f& maxc) const
{
    minc[0] = float(_vbnd[0]);
    minc[1] = float(_vbnd[2]);
    minc[2] = float(_vbnd[4]);
    maxc[0] = float(_vbnd[1]);
    maxc[1] = float(_vbnd[3]);
    maxc[2] = float(_vbnd[5]);
}   // end corners


cv::Vec3f ObjModelBounds::centre() const
{
    return cv::Vec3f( float((_vbnd[0] + _vbnd[1]) * 0.5), float((_vbnd[2] + _vbnd[3]) * 0.5), float((_vbnd[4] + _vbnd[5]) * 0.5));
}   // end centre


bool ObjModelBounds::intersects( const ObjModelBounds& ob) const
{
    cv::Vec3f min0c, max0c;
    corners( min0c, max0c);

    cv::Vec3f min1c, max1c;
    ob.corners( min1c, max1c);

    const cv::Vec6d a( min0c[0], max0c[0], min0c[1], max0c[1], min0c[2], max0c[2]);
    const cv::Vec6d b( min1c[0], max1c[0], min1c[1], max1c[1], min1c[2], max1c[2]);

    const bool xAinB = ((a[0] >= b[0] && a[0] <= b[1]) || (a[1] >= b[0] && a[1] <= b[1]));    // x edges of A in B
    const bool xBinA = ((b[0] >= a[0] && b[0] <= a[1]) || (b[1] >= a[0] && b[1] <= a[1]));    // x edges of B in A
    const bool yAinB = ((a[2] >= b[2] && a[2] <= b[3]) || (a[3] >= b[2] && a[3] <= b[3]));    // y edges of A in B
    const bool yBinA = ((b[2] >= a[2] && b[2] <= a[3]) || (b[3] >= a[2] && b[3] <= a[3]));    // y edges of B in A
    const bool zAinB = ((a[4] >= b[4] && a[4] <= b[5]) || (a[5] >= b[4] && a[5] <= b[5]));    // y edges of A in B
    const bool zBinA = ((b[4] >= a[4] && b[4] <= a[5]) || (b[5] >= a[4] && b[5] <= a[5]));    // y edges of B in A

    const bool xint = xAinB || xBinA;
    const bool yint = yAinB || yBinA;
    const bool zint = zAinB || zBinA;

    return xint && yint && zint;
}   // end intersects


// static
cv::Vec6d ObjModelBounds::as6d( const cv::Vec3f& minc, const cv::Vec3f& maxc)
{
    return cv::Vec6d( minc[0], maxc[0], minc[1], maxc[1], minc[2], maxc[2]);
}   // end as6d
