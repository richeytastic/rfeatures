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

#include <ObjModelSurfacePlanePathFinder.h>
#include <ObjModelSurfacePointFinder.h>
#include <FeatureUtils.h>
#include <ObjPolyPlane.h>
#include <deque>
#include <cassert>
using RFeatures::ObjModelSurfacePlanePathFinder;
using RFeatures::ObjModelSurfacePathFinder;
using RFeatures::ObjModelKDTree;
using RFeatures::ObjModel;

namespace {


class FixedPlaneSlicingPath
{
public:
    // Point p must be in the plane of polygon fid. Vector u defines the slicing plane's orientation.
    FixedPlaneSlicingPath( const ObjModel& m, int fid, const cv::Vec3f& p, const cv::Vec3f& u)
        : _model(m), _u(u), _ffid(fid), _bfid(fid)
    {
        _evtxs.push_back(p);
        _iidx = 0;
    }   // end ctor


    void init()
    {
        assert( _evtxs.size() == 1);
        assert( _ffid == _bfid);
        const int fid = _ffid;

        const RFeatures::ObjPolyPlane pp( _model, fid, _evtxs.back(), _u);
        assert( pp.inhalf() == 0);
        _ffid = _model.oppositePoly( fid, pp.vaid(), pp.vbid());
        if ( _ffid >= 0)
        {
            _evtxs.push_front( pp.abIntersection());
            _iidx = 1;
        }   // end if
        _bfid = _model.oppositePoly( fid, pp.vaid(), pp.vcid());
        if ( _bfid >= 0)
            _evtxs.push_back( pp.acIntersection());
    }   // end init

/*
    void print( const std::string& pfx) const
    {
        std::cerr << pfx << ":";
        for ( const cv::Vec3f& v : _evtxs)
            std::cerr << " " << v ;
        std::cerr << " [" << _evtxs.size() << "]  _ffid: " << _ffid << ", _bfid: " << _bfid << std::endl;
    }   // end print
*/


    bool canSplice( const FixedPlaneSlicingPath& fpsp) const
    {
        return (frontPoly() >= 0 && (fpsp.frontPoly() == frontPoly() || fpsp.backPoly() == frontPoly())) ||
               (backPoly() >= 0 && (fpsp.frontPoly() == backPoly() || fpsp.backPoly() == backPoly()));
    }   // end canSplice


    void splice( const FixedPlaneSlicingPath& fpsp, std::vector<cv::Vec3f>& path) const
    {
        path.clear();

        // Is it the front part of this path that can be spliced on or the back?
        if ( frontPoly() >= 0 && (frontPoly() == fpsp.frontPoly() || frontPoly() == fpsp.backPoly()))
            _pushOnInitialToFront( path);
        else if ( backPoly() >= 0 && (backPoly() == fpsp.frontPoly() || backPoly() == fpsp.backPoly()))
            _pushOnInitialToBack( path);

        // And is it the front part of the other path than can be spliced on, or the back?
        if ( fpsp.frontPoly() >= 0 && (fpsp.frontPoly() == frontPoly() || fpsp.frontPoly() == backPoly()))
            fpsp._pushOnFrontToInitial( path);
        else if ( fpsp.backPoly() >= 0 && (fpsp.backPoly() == frontPoly() || fpsp.backPoly() == backPoly()))
            fpsp._pushOnBackToInitial( path);
    }   // end splice


    // Returns true iff this path can be extended at one or both of its ends.
    bool canExtend() const { return _ffid >= 0 || _bfid >= 0;}


    // Returns true if can keep extending the front or back of the path.
    bool extend()
    {
        bool dexFront = false;
        bool dexBack = false;
        if ( _ffid >= 0)
            dexFront = extendFront();
        if ( _bfid >= 0)
            dexBack = extendBack();
        return dexFront || dexBack;
    }   // end extend


    bool extendFront()
    {
        assert( _ffid >= 0);
        const cv::Vec3f& v = _evtxs.front();
        const RFeatures::ObjPolyPlane pp( _model, _ffid, v, _u);
        // ObjPolyPlane::inhalf must return 0 because p is inside the bounds of polygon fid.
        // Even if p is at one vertex, it must still logically cross at that point.
        assert( pp.inhalf() == 0);

        // Get the correct intersecting edge vertex
        cv::Vec3f ev = pp.abIntersection(); // Assume its on edge ab
        const cv::Vec3f vac = pp.acIntersection();
        if ( RFeatures::l2sq( vac - v) < 1e-5)  // But if it's the previous vertex, then it must be the other edge that intersects
            _ffid = _model.oppositePoly( _ffid, pp.vaid(), pp.vbid());    // Returns -1 if ab is an edge
        else
        {
            _ffid = _model.oppositePoly( _ffid, pp.vaid(), pp.vcid());    // Returns -1 if ac is an edge
            ev = vac;
        }   // end else

        if ( _ffid >= 0)
        {
            // If the opposite poly is the back poly then we've encountered a loop so fail extending the front
            if ( _ffid == _bfid)
                _ffid = -1;
            else
            {
                _evtxs.push_front( ev);
                _iidx++;
            }   // end else
        }   // end if
        return _ffid >= 0;
    }   // end extendFront


    bool extendBack()
    {
        assert( _bfid >= 0);
        const cv::Vec3f& v = _evtxs.back();
        const RFeatures::ObjPolyPlane pp( _model, _bfid, v, _u);
        assert( pp.inhalf() == 0);

        cv::Vec3f ev = pp.abIntersection();
        const cv::Vec3f vac = pp.acIntersection();
        if ( RFeatures::l2sq( vac - v) < 1e-5)  // But if it's the previous vertex, then it must be the other edge that intersects
            _bfid = _model.oppositePoly( _bfid, pp.vaid(), pp.vbid());
        else
        {
            _bfid = _model.oppositePoly( _bfid, pp.vaid(), pp.vcid());
            ev = vac;
        }   // end else

        if ( _bfid >= 0)
        {
            // If the opposite poly is the front poly then we've encountered a loop so fail extending the back
            if ( _bfid == _ffid)
                _bfid = -1;
            else
                _evtxs.push_back( ev);
        }   // end if
        return _bfid >= 0;
    }   // end extendBack

private:
    const ObjModel& _model;
    const cv::Vec3f _u;
    int _ffid;  // Front polygon index
    int _bfid;  // Back polygon index
    std::deque<cv::Vec3f> _evtxs;    // Edge crossing vertices
    int _iidx;  // Index into _evtxs of the initial vertex

    int frontPoly() const { return _ffid;}
    int backPoly() const { return _bfid;}


    void _checkPushBack( std::vector<cv::Vec3f>& path, int iidx) const
    {
        const cv::Vec3f& nv = _evtxs[iidx];
        if ( path.empty())
            path.push_back( nv);
        else
        {
            const cv::Vec3f& cv = path.back();
            if ( RFeatures::l2sq(nv-cv) >= 1e-6)
                path.push_back( nv);
        }   // end else
    }   // end _checkPushBack


    void _pushOnInitialToFront( std::vector<cv::Vec3f>& path) const
    {
        int iidx = _iidx;
        while ( iidx >= 0)
        {
            _checkPushBack( path, iidx);
            iidx--;
        }   // end while
    }   // _pushOnInitialToFront


    void _pushOnInitialToBack( std::vector<cv::Vec3f>& path) const
    {
        int iidx = _iidx;
        const int n = static_cast<int>(_evtxs.size());
        while ( iidx < n)
        {
            _checkPushBack( path, iidx);
            iidx++;
        }   // end while
    }   // _pushOnInitialToBack


    void _pushOnFrontToInitial( std::vector<cv::Vec3f>& path) const
    {
        int iidx = 0;
        while ( iidx <= _iidx)
        {
            _checkPushBack( path, iidx);
            iidx++;
        }   // end while
    }   // _pushOnFrontToInitial


    void _pushOnBackToInitial( std::vector<cv::Vec3f>& path) const
    {
        int iidx = static_cast<int>(_evtxs.size()) - 1;
        while ( iidx >= _iidx)
        {
            _checkPushBack( path, iidx);
            iidx--;
        }   // end while
    }   // _pushOnBackToInitial
};  // end class

}   // end namespace


ObjModelSurfacePlanePathFinder::ObjModelSurfacePlanePathFinder( const ObjModel& m, const ObjModelKDTree& k, bool fs)
    : ObjModelSurfacePathFinder(m, k), _findShortest(fs), _fixedPlane(false), _u(0,0,0) {}


ObjModelSurfacePlanePathFinder::ObjModelSurfacePlanePathFinder( const ObjModel& m, const ObjModelKDTree& k, const cv::Vec3f& u, bool fs)
    : ObjModelSurfacePathFinder(m, k), _findShortest(fs), _fixedPlane(true), _u(u) {}


cv::Vec3f ObjModelSurfacePlanePathFinder::_findInitialVertex( const cv::Vec3f& pos, int &fid) const
{
    ObjModelSurfacePointFinder spf(_model);
    int vid = _kdt.find(pos);
    cv::Vec3f v;
    spf.find( pos, vid, fid, v);
    return v;
}   // end _findInitialVertex


double ObjModelSurfacePlanePathFinder::findPath( const cv::Vec3f& spos, const cv::Vec3f& fpos)
{
    _lpath.clear(); // protected

    // Find the start and finish end points on the surface of the model as v0 and v1 respectively.
    int f0, f1;
    const cv::Vec3f v0 = _findInitialVertex( spos, f0);
    const cv::Vec3f v1 = _findInitialVertex( fpos, f1);

    const cv::Vec3f pvec = v1 - v0;  // Path vector slicing plane calculation.

    // If the start and end vertices are on the same polygon, the returned list of points is simply the start and end vertex
    if ( f0 == f1)
    {
        _lpath.push_back(v0);
        _lpath.push_back(v1);
        return cv::norm(pvec);
    }   // end if

    // Parameters to define the slicing plane (fixed or local i.e. per polygon).
    cv::Vec3f n;
    cv::normalize( _u.cross(pvec), n); // Default to fixed slicing plane. If non-fixed, n is set differently for each polygon based on its normal.
    /*
    if ( !_fixedPlane)
    {
        const cv::Vec3d fn = _model.calcFaceNorm(f0);
        cv::normalize( fn.cross(pvec), n);
    }   // end if
    */

    // Maintain two lists holding the sequences of polygons crossed by the slicing plane. When these two lists
    // can be joined because a pair of endpoints share a polygon ID then the path is found. There may be a second
    // possibly shorter path between these two points and if _findShortest is true, the algorithm will continue
    // until either this shorter path is found, or there is no other path found (a model edge is found).
    // If both ends of either list either loops back onto itself, or hits a dead-end (an edge) then no path is found.
    FixedPlaneSlicingPath sp0( _model, f0, v0, n);
    sp0.init();

    FixedPlaneSlicingPath sp1( _model, f1, v1, n);
    if ( !sp0.canSplice( sp1))
        sp1.init();

    while ( !sp0.canSplice(sp1) && sp0.canExtend() && sp1.canExtend())
    {
        sp0.extend();
        if ( !sp0.canSplice(sp1))
            sp1.extend();
    }   // end while

    // Joined up path found?
    if ( sp0.canSplice(sp1))
        sp0.splice( sp1, _lpath);

    return _calcPathLength();   // Protected
}   // end findPath
