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

struct PolyFinder
{
    PolyFinder( const ObjModel& model, const cv::Vec3f& t) : _model(model), _t(t), _minsd(DBL_MAX), _bv(t), _bfid(-1)
    {
        _sfids.insert(-1);  // Because oppositePoly returns -1 if no opposite poly found.
    }   // end ctor

    double find( int fid)
    {
        _sfids.insert( fid);   // Don't check this face again
        const cv::Vec3d u = _model.projectToPoly( fid, _t);    // Project t into polygon fid
        const double sd = RFeatures::l2sq( u - _t); // Repositioned difference

        if ( sd <= (_minsd + 1e-12))    // At least as close to t on repositioning (plus a tiny term due to rounding errors)
        {
            _minsd = std::min( sd, _minsd);   // Due to possible rounding error
            _bv = u;
            _bfid = fid;

            const int* vidxs = _model.fvidxs(fid); // Check adjacent polygons

            const int f0 = _model.oppositePoly( fid, vidxs[0], vidxs[1]);
            if ( _sfids.count(f0) == 0 && _model.isVertexInsideFace(f0,u))
                find( f0);

            const int f1 = _model.oppositePoly( fid, vidxs[1], vidxs[2]);
            if ( _sfids.count(f1) == 0 && _model.isVertexInsideFace(f1,u))
                find( f1);

            const int f2 = _model.oppositePoly( fid, vidxs[2], vidxs[0]);
            if ( _sfids.count(f2) == 0 && _model.isVertexInsideFace(f2,u))
                find( f2);
        }   // end if

        return _minsd;
    }   // end find

    cv::Vec3f vertex() const { return cv::Vec3f( static_cast<float>(_bv[0]), static_cast<float>(_bv[1]), static_cast<float>(_bv[2]));}

    int poly() const { return _bfid;}

private:
    const ObjModel& _model;
    const cv::Vec3d _t;
    double _minsd;
    cv::Vec3d _bv;
    int _bfid;
    IntSet _sfids;
};  // end struct

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
        PolyFinder pfinder( _model, t);
        sd = pfinder.find( fid);
        fv = pfinder.vertex();
        fid = pfinder.poly();
        vidx = -1;
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

