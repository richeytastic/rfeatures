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

ObjModelSurfacePointFinder::ObjModelSurfacePointFinder( const ObjModel* m) : _model(m) {}


namespace {

void findClosestSurface( const ObjModel* model, const cv::Vec3f& t, int vidx, IntSet& vfaces, int& bfid, cv::Vec3f& fv, double &minsd)
{
    // The next vertices to check
    int nv0 = vidx;
    int nv1 = vidx;

    const IntSet& fids = model->getFaceIds(vidx);
    for ( int fid : fids)
    {
        if ( vfaces.count(fid) > 0)
            continue;

        vfaces.insert(fid);   // Don't check this face again
        const cv::Vec3f u = model->projectToPoly( fid, t);    // Project t into polygon fid
        const double sd = RFeatures::l2sq(u-t); // Repositioned difference
        if ( sd < minsd)    // Closer to t on repositioning?
        {
            minsd = sd;
            fv = u;
            model->poly(fid).opposite( vidx, nv0, nv1); // Get the opposite two vertices to check next.
            bfid = fid;
        }   // end if
    }   // end foreach

    if ( nv0 != vidx)
        findClosestSurface( model, t, nv0, vfaces, bfid, fv, minsd);

    if ( nv1 != vidx)
        findClosestSurface( model, t, nv1, vfaces, bfid, fv, minsd);
}   // end findClosestSurface

}   // end namespace


// public
double ObjModelSurfacePointFinder::find( cv::Vec3f t, int& vidx, int& bfid, cv::Vec3f& fv) const
{
    double sd = 0;
    bfid = *_model->getFaceIds(vidx).begin();
    // Check if vertex at vidx at same location as t
    if ( _model->vtx(vidx) == t)
        fv = t;
    else
    {
        IntSet vfaces;  // Visited polygons
        fv = t;
        sd = DBL_MAX;
        findClosestSurface( _model, t, vidx, vfaces, bfid, fv, sd);
        vidx = -1;
    }   // end else
    return sd;
}   // end find

