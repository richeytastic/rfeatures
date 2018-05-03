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

#include <ObjModelSurfacePointFinder.h>
#include <FeatureUtils.h>       // l2sq
using RFeatures::ObjModelSurfacePointFinder;
using RFeatures::ObjModel;

ObjModelSurfacePointFinder::ObjModelSurfacePointFinder( const ObjModel::Ptr m)
    : _model(m)
{}


namespace {

double findClosestSurface( const RFeatures::ObjModel::Ptr model, const cv::Vec3f& v, int vidx, IntSet& visitedFaces, int& bfid, cv::Vec3f& fv)
{
    double minsd = RFeatures::l2sq(fv - v);

    // The next vertices to check
    int nv0 = vidx;
    int nv1 = vidx;

    const IntSet& fids = model->getFaceIds(vidx);
    for ( int fid : fids)
    {
        if ( visitedFaces.count(fid) > 0)
            continue;

        visitedFaces.insert(fid);   // Don't check this face again
        const cv::Vec3f u = model->projectToPoly( fid, v);    // Project v into polygon fid
        const double sd = RFeatures::l2sq(u-v); // Repositioned difference
        if ( sd < minsd)
        {
            minsd = sd;
            fv = u;
            model->getFace(fid).getOpposite( vidx, nv0, nv1); // Get the opposite two vertices to check next.
            bfid = fid;
        }   // end if
    }   // end foreach

    if ( nv0 != vidx)
        minsd = findClosestSurface( model, v, nv0, visitedFaces, bfid, fv);

    if ( nv1 != vidx)
        minsd = findClosestSurface( model, v, nv1, visitedFaces, bfid, fv);

    return minsd;
}   // end findClosestSurface

}   // end namespace


// public
double ObjModelSurfacePointFinder::find( const cv::Vec3f& v, int& vidx, int& bfid, cv::Vec3f& fv) const
{
    double sd;
    // Check if vertex at vidx at same location as v
    if ( _model->vtx(vidx) == v)
    {
        bfid = -1;  // Denote that v is not in the plane of any of the polygons attached to vidx.
        fv = _model->vtx(vidx);
        sd = l2sq(fv - v);
    }   // end if
    else
    {
        IntSet visitedFaces;
        fv = cv::Vec3f( 10e4, 10e4, 10e4) + v;
        sd = findClosestSurface( _model, v, vidx, visitedFaces, bfid, fv);
        vidx = -1;
    }   // end else
    return sd;
}   // end find

