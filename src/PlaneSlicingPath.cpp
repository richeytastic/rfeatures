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

#include <PlaneSlicingPath.h>
#include <ObjPolyPlane.h>
#include <cassert>
using RFeatures::PlaneSlicingPath;
using RFeatures::ObjModel;


PlaneSlicingPath::PlaneSlicingPath( const ObjModel& m, int ifid, const cv::Vec3f& ip)
    : _model(m), _ip(ip), _ifid(ifid)
{
    reset();
}   // end ctor


void PlaneSlicingPath::reset()
{
    _ffid = -1;
    _nfid = _ifid;
    _pfids.clear();
    _evtxs.clear();
    _evtxs.push_back(_ip);
    _pfids.insert(_ifid);
}   // end reset


void PlaneSlicingPath::init( int notThisFid)
{
    reset();

    // Derived class implementation defines the calculation to find the plane that
    // crosses at right angles to the path direction and goes through triangle _ifid.
    const cv::Vec3f u = polySlicingPlane( _ifid, _ip);
    const RFeatures::ObjPolyPlane pp( _model, _ifid, _ip, u);
    assert( pp.inhalf() == 0);

    int nfid = -1;

    const int nfidab = _model.oppositePoly( _ifid, pp.vaid(), pp.vbid());
    if ( nfidab >= 0 && nfidab != notThisFid)
    {
        nfid = nfidab;
        _evtxs.push_back( pp.abIntersection());
    }   // end if

    const int nfidac = _model.oppositePoly( _ifid, pp.vaid(), pp.vcid());
    if ( nfid < 0 && nfidac >= 0 && nfidac != notThisFid)
    {
        nfid = nfidac;
        _evtxs.push_back( pp.acIntersection());
    }   // end else

    _nfid = _ffid = nfid;
}   // end init


int PlaneSlicingPath::nextPoly() const { return _nfid;}
bool PlaneSlicingPath::canSplice( const PlaneSlicingPath& psp) const { return _nfid >= 0 && psp.nextPoly() == _nfid;}


void PlaneSlicingPath::_pushOnInitialToBack( std::vector<cv::Vec3f>& path) const
{
    int iidx = 0;
    const int n = static_cast<int>(_evtxs.size());
    while ( iidx < n)
        path.push_back( _evtxs[iidx++]);
}   // _pushOnInitialToBack


void PlaneSlicingPath::_pushOnBackToInitial( std::vector<cv::Vec3f>& path) const
{
    int iidx = static_cast<int>(_evtxs.size()) - 1;
    while ( iidx >= 0)
        path.push_back( _evtxs[iidx--]);
}   // _pushOnBackToInitial


void PlaneSlicingPath::splice( const PlaneSlicingPath& psp, std::vector<cv::Vec3f>& path) const
{
    path.clear();
    if ( canSplice(psp))
    {
        _pushOnInitialToBack( path);
        psp._pushOnBackToInitial( path);
    }   // end if
}   // end splice


bool PlaneSlicingPath::canExtend() const { return _nfid >= 0;}


int PlaneSlicingPath::_findNextPolyEdgeVertex( int fid, const cv::Vec3f& v, cv::Vec3f& ev)
{
    _pfids.insert(fid); // Record that we've parsed this poly
    const RFeatures::ObjPolyPlane pp( _model, fid, v, polySlicingPlane(fid, v));
    // inhalf must return 0 because p is inside the bounds of polygon fid.
    // Even if p is at one vertex, it must still logically cross at that point.
    assert( pp.inhalf() == 0);
    int nfid = _model.oppositePoly( fid, pp.vaid(), pp.vbid());
    if ( _pfids.count(nfid) == 0)
        ev = pp.abIntersection();
    else
    {
        nfid = _model.oppositePoly( fid, pp.vaid(), pp.vcid());
        ev = pp.acIntersection();
    }   // end else
    // If we've already parsed nfid, then return -1, otherwise return nfid (which could still be -1 if an edge was reached)
    return _pfids.count(nfid) > 0 ? -1 : nfid;
}   // end _findNextPolyEdgeVertex


bool PlaneSlicingPath::extend()
{
    assert( _nfid >= 0);
    cv::Vec3f ev;
    _nfid = _findNextPolyEdgeVertex( _nfid, _evtxs.back(), ev);

    if ( _nfid >= 0)
    {
        // If the opposite poly is the initial poly then we've encountered a loop so fail
        if ( _nfid == _ifid)
            _nfid = -1;
        else
            _evtxs.push_back( ev);
    }   // end if

    return _nfid >= 0;
}   // end extend
