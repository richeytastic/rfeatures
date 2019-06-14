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
using RFeatures::ObjModelBounds;
using RFeatures::ObjModel;
using RFeatures::ObjPoly;
#include <algorithm>
#include <iomanip>
#include <cassert>

namespace {

void expandBounds( const ObjModel* model, int vidx, cv::Vec6i& bounds)
{
    const cv::Vec3f& newv = model->vtx(vidx);

    if ( newv[0] < model->vtx(bounds[0])[0])    // Xmin
        bounds[0] = vidx;
    if ( newv[0] > model->vtx(bounds[1])[0])    // Xmax
        bounds[1] = vidx;

    if ( newv[1] < model->vtx(bounds[2])[1])    // Ymin
        bounds[2] = vidx;
    if ( newv[1] > model->vtx(bounds[3])[1])    // Ymax
        bounds[3] = vidx;

    if ( newv[2] < model->vtx(bounds[4])[2])    // Zmin
        bounds[4] = vidx;
    if ( newv[2] > model->vtx(bounds[5])[2])    // Zmax
        bounds[5] = vidx;
}   // end expandBounds


// Return the indices of the vertices from the provided set that bound all of the vertices in that set.
// The order of the vertex indices returned is [Xmin, Xmax, Ymin, Ymax, Zmin, Zmax].
cv::Vec6i findVertexBounds( const ObjModel* model, const IntSet& vidxs)
{
    cv::Vec6i bounds;
    bounds[0] = *vidxs.begin();  // Xmin
    bounds[1] = bounds[0];  // Xmax
    bounds[2] = bounds[0];  // Ymin
    bounds[3] = bounds[0];  // Ymax
    bounds[4] = bounds[0];  // Zmin
    bounds[5] = bounds[0];  // Zmax
    std::for_each( std::begin(vidxs), std::end(vidxs), [&](int v){ expandBounds( model, v, bounds);});
    return bounds;
}   // end findVertexBounds


void asMinMax( const ObjModel* model, const cv::Vec6i& vb, cv::Vec3f& minc, cv::Vec3f& maxc)
{
    minc[0] = model->vtx(vb[0])[0];
    minc[1] = model->vtx(vb[2])[1];
    minc[2] = model->vtx(vb[4])[2];
    maxc[0] = model->vtx(vb[1])[0];
    maxc[1] = model->vtx(vb[3])[1];
    maxc[2] = model->vtx(vb[5])[2];
}   // end asMinMax

}   // end namespace


ObjModelBounds::Ptr ObjModelBounds::create( const ObjModel* model, const IntSet* vset)
{
    return Ptr( new ObjModelBounds( model, vset));
}   // end create


ObjModelBounds::Ptr ObjModelBounds::create( const ObjModel* model, const IntSet& pset)
{
    return Ptr( new ObjModelBounds( model, pset));
}   // end create


ObjModelBounds::ObjModelBounds( const ObjModel* model, const IntSet* vset)
    : _model(model)
{
    if ( !vset)
    {
        _vbnd = {0,0,0,0,0,0};
        const IntSet& vids = model->vtxIds();
        for ( int v : vids)
            expandBounds( model, v, _vbnd);
    }   // end if
    else
        _vbnd = findVertexBounds( model, *vset);
}   // end ctor


ObjModelBounds::ObjModelBounds( const ObjModel* model, const IntSet& pset)
    : _model(model)
{
    IntSet vset;
    for ( int fid : pset)
    {
        const ObjPoly& p = model->face(fid);
        vset.insert(p[0]);
        vset.insert(p[1]);
        vset.insert(p[2]);
    }   // end for
    _vbnd = findVertexBounds( model, vset);
}   // end ctor


void ObjModelBounds::corners( cv::Vec3f& minc, cv::Vec3f& maxc) const
{
    asMinMax( _model, _vbnd, minc, maxc);
}   // end corners


bool ObjModelBounds::intersects( const ObjModelBounds& ob) const
{
    cv::Vec3f min0c, max0c;
    corners( min0c, max0c);

    cv::Vec3f min1c, max1c;
    ob.corners( min1c, max1c);

    const cv::Vec6d a( min0c[0], max0c[0], min0c[1], max0c[1], min0c[2], max0c[2]);
    const cv::Vec6d b( min1c[0], max1c[0], min1c[1], max1c[1], min1c[2], max1c[2]);

    bool xAinB = ((a[0] >= b[0] && a[0] <= b[1]) || (a[1] >= b[0] && a[1] <= b[1]));    // x edges of A in B
    bool xBinA = ((b[0] >= a[0] && b[0] <= a[1]) || (b[1] >= a[0] && b[1] <= a[1]));    // x edges of B in A
    bool yAinB = ((a[2] >= b[2] && a[2] <= b[3]) || (a[3] >= b[2] && a[3] <= b[3]));    // y edges of A in B
    bool yBinA = ((b[2] >= a[2] && b[2] <= a[3]) || (b[3] >= a[2] && b[3] <= a[3]));    // y edges of B in A
    bool zAinB = ((a[4] >= b[4] && a[4] <= b[5]) || (a[5] >= b[4] && a[5] <= b[5]));    // y edges of A in B
    bool zBinA = ((b[4] >= a[4] && b[4] <= a[5]) || (b[5] >= a[4] && b[5] <= a[5]));    // y edges of B in A

    bool xint = xAinB || xBinA;
    bool yint = yAinB || yBinA;
    bool zint = zAinB || zBinA;

    return xint && yint && zint;
}   // end intersects


cv::Vec6d ObjModelBounds::as6d() const
{
    cv::Vec3f minc, maxc;
    corners( minc, maxc);
    return ObjModelBounds::as6d( minc, maxc);
}   // end as6d


cv::Vec3f ObjModelBounds::centre() const
{
    cv::Vec3f minc, maxc;
    corners( minc, maxc);
    return ObjModelBounds::centre( minc, maxc);
}   // end centre


double ObjModelBounds::diagonal() const
{
    cv::Vec3f minc, maxc;
    corners( minc, maxc);
    return ObjModelBounds::diagonal( minc, maxc);
}   // end diagonal


double ObjModelBounds::xlen() const
{
    cv::Vec3f minc, maxc;
    corners( minc, maxc);
    return maxc[0] - minc[0];
}   // end xlen


double ObjModelBounds::ylen() const
{
    cv::Vec3f minc, maxc;
    corners( minc, maxc);
    return maxc[1] - minc[1];
}   // end ylen


double ObjModelBounds::zlen() const
{
    cv::Vec3f minc, maxc;
    corners( minc, maxc);
    return maxc[2] - minc[2];
}   // end zlen


cv::Vec6d ObjModelBounds::as6d( const cv::Vec3f& minc, const cv::Vec3f& maxc)
{
    return cv::Vec6d( minc[0], maxc[0], minc[1], maxc[1], minc[2], maxc[2]);
}   // end as6d
