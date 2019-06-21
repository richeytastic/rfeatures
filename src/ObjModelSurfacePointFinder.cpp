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

#include <ObjModelTools.h>
using RFeatures::ObjModelSurfacePointFinder;
using RFeatures::ObjModel;

namespace {

void findClosestSurface( const ObjModel& model, const cv::Vec3d& t, IntSet& vfids, int& fid, cv::Vec3d& v, double &minsd)
{
    vfids.insert(fid);   // Don't check this face again
    const int infid = fid;

    const cv::Vec3d u = model.projectToPoly( fid, t);    // Project t into polygon fid
    const double sd = RFeatures::l2sq(u-t); // Repositioned difference
    if ( sd <= minsd)    // At least as clos to t on repositioning?
    {
        minsd = sd;
        v = u;

        // Get the next poly to check
        const int* vidxs = model.fvidxs(fid);
        const int f0 = model.oppositePoly( fid, vidxs[0], vidxs[1]);
        const int f1 = model.oppositePoly( fid, vidxs[1], vidxs[2]);
        const int f2 = model.oppositePoly( fid, vidxs[2], vidxs[0]);

        if ( vfids.count(f0) == 0 && model.isVertexInsideFace(f0,u))
            fid = f0;
        else if ( vfids.count(f1) == 0 && model.isVertexInsideFace(f1,u))
            fid = f1;
        else if ( vfids.count(f2) == 0 && model.isVertexInsideFace(f2,u))
            fid = f2;
    }   // end if

    if ( fid != infid)
        findClosestSurface( model, t, vfids, fid, v, minsd);
}   // end findClosestSurface

}   // end namespace


// public
double ObjModelSurfacePointFinder::find( cv::Vec3f ft, int& vidx, int& fid, cv::Vec3f& fv) const
{
    double sd = 0;
    fid = *_model.faces(vidx).begin();
    // Check if vertex at vidx at same location as t
    if ( _model.vtx(vidx) == ft)
        fv = ft;
    else
    {
        IntSet vfids;  // Visited faces
        vfids.insert(-1);
        sd = DBL_MAX;
        cv::Vec3d v(0,0,0);
        const cv::Vec3d t = ft;
        findClosestSurface( _model, t, vfids, fid, v, sd);
        fv = v;
        vidx = -1;
    }   // end else
    return sd;
}   // end find

