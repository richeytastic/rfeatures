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

int findClosestPoly( const ObjModel& model, const cv::Vec3d& t, IntSet& vfids, int fid, cv::Vec3d& v, double &minsd)
{
    vfids.insert(fid);   // Don't check this face again
    const cv::Vec3d u = model.projectToPoly( fid, t);    // Project t into polygon fid
    const double sd = RFeatures::l2sq(u-t); // Repositioned difference

    if ( sd <= (minsd + 1e-12))    // At least as close to t on repositioning (plus a tiny term due to rounding errors)
    {
        minsd = std::min( sd, minsd);   // Due to possible rounding error
        v = u;

        const int* vidxs = model.fvidxs(fid); // Check adjacent polygons

        const int f0 = model.oppositePoly( fid, vidxs[0], vidxs[1]);
        const int f1 = model.oppositePoly( fid, vidxs[1], vidxs[2]);
        const int f2 = model.oppositePoly( fid, vidxs[2], vidxs[0]);

        if ( vfids.count(f0) == 0 && model.isVertexInsideFace(f0,u))
            fid = findClosestPoly( model, t, vfids, f0, v, minsd);

        if ( vfids.count(f1) == 0 && model.isVertexInsideFace(f1,u))
            fid = findClosestPoly( model, t, vfids, f1, v, minsd);

        if ( vfids.count(f2) == 0 && model.isVertexInsideFace(f2,u))
            fid = findClosestPoly( model, t, vfids, f2, v, minsd);
    }   // end if

    return fid;
}   // end findClosestPoly

}   // end namespace


double ObjModelSurfacePointFinder::find( const cv::Vec3f& t, int& vidx, int& fid, cv::Vec3f& fv) const
{
    assert( !_model.faces(vidx).empty());
    fid = *_model.faces(vidx).begin();
    double sd = 0;
    if ( _model.vtx(vidx) == t)
        fv = t;
    else
    {
        vidx = -1;
        IntSet vfids;       // Visited faces
        vfids.insert(-1);   // Because oppositePoly returns -1 if no opposite poly found.
        sd = DBL_MAX;
        const cv::Vec3d td = t;
        cv::Vec3d fvd = fv;
        fid = findClosestPoly( _model, td, vfids, fid, fvd, sd);
        fv[0] = static_cast<float>(fvd[0]);
        fv[1] = static_cast<float>(fvd[1]);
        fv[2] = static_cast<float>(fvd[2]);
    }   // end else
    return sd;
}   // end find


cv::Vec3f ObjModelSurfacePointFinder::find( const cv::Vec3f& t, int vidx, int* fid) const
{
    int f = 0;
    cv::Vec3f fv = t;
    if ( vidx < 0)
        vidx = *_model.vtxIds().begin();
    find( t, vidx, f, fv);
    if ( fid)
        *fid = f;
    return fv;
}   // end find

