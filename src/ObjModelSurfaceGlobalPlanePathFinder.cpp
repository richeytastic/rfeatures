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

#include <ObjModelSurfaceGlobalPlanePathFinder.h>
#include <ObjModelSurfacePointFinder.h>
#include <FeatureUtils.h>
using RFeatures::ObjModelSurfaceGlobalPlanePathFinder;
using RFeatures::ObjModelSurfacePathFinder;
using RFeatures::GlobalPlaneSlicingPath;
using RFeatures::ObjModelKDTree;
using RFeatures::ObjModel;


ObjModelSurfaceGlobalPlanePathFinder::ObjModelSurfaceGlobalPlanePathFinder( const ObjModel& m, const ObjModelKDTree& k, const cv::Vec3f& u)
    : ObjModelSurfacePathFinder(m, k), _u(u)
{
    assert( l2sq(u) > 0);   // Cannot be zero vector
}   // end ctor


cv::Vec3f RFeatures::findInitialVertex( const ObjModel& model, const ObjModelKDTree& kdt, const cv::Vec3f& pos, int &fid)
{
    ObjModelSurfacePointFinder spf(model);
    int vid = kdt.find(pos);
    cv::Vec3f v;
    spf.find( pos, vid, fid, v);
    return v;
}   // end findInitialVertex


double RFeatures::findPathOption( PlaneSlicingPath& sp0, int nfid0, PlaneSlicingPath& sp1, int nfid1, std::vector<cv::Vec3f>& path)
{
    sp0.init( nfid0);
    if ( !sp0.canSplice( sp1))
        sp1.init( nfid1);

    while ( !sp0.canSplice(sp1) && sp0.canExtend() && sp1.canExtend())
    {
        sp0.extend();
        if ( !sp0.canSplice(sp1))
            sp1.extend();
    }   // end while

    double psum = 0;
    // Joined up path found?
    if ( sp0.canSplice(sp1))
    {
        sp0.splice( sp1, path);
        psum = ObjModelSurfacePathFinder::calcPathLength( path);
    }   // end if

    return psum;
}   // end findPathOption


std::vector<cv::Vec3f> RFeatures::findBestPath( PlaneSlicingPath& sp0, PlaneSlicingPath& sp1)
{
    // There are four possible paths to take from the two endpoints.
    std::vector<cv::Vec3f> path0, path1;

    double psum0 = findPathOption( sp0, -1, sp1, -1, path0);
    const int sp0Afid = sp0.firstPoly();    // First polygon that path took from v0 (direction from f0)
    const int sp1Afid = sp1.firstPoly();    // First polygon that path took from v1 (direction from f1)

    double psum1 = findPathOption( sp0, sp0Afid, sp1, sp1Afid, path1);  // Choose a different direction to go in for the endpoint
    const int sp0Bfid = sp0.firstPoly();    // Second polygon direction that path took from v0 (direction from f0)
    const int sp1Bfid = sp1.firstPoly();    // Second polygon direction that path took from v1 (direction from f1)

    if ( psum0 == 0)
        psum0 = findPathOption( sp0, sp0Bfid, sp1, sp1Afid, path0);
    else if ( psum1 == 0)
        psum1 = findPathOption( sp0, sp0Bfid, sp1, sp1Afid, path1);
    
    if ( psum0 == 0)
        psum0 = findPathOption( sp0, sp0Afid, sp1, sp1Bfid, path0);
    else if ( psum1 == 0)
        psum1 = findPathOption( sp0, sp0Bfid, sp1, sp1Afid, path1);

    if ( psum0 == 0)
        psum0 = DBL_MAX;
    if ( psum1 == 0)
        psum1 = DBL_MAX;

    return psum0 < psum1 ? path0 : path1;
}   // end findBestPath


double ObjModelSurfaceGlobalPlanePathFinder::findPath( const cv::Vec3f& spos, const cv::Vec3f& fpos)
{
    // Find the start and finish end points on the surface of the model as v0 and v1 respectively.
    int f0, f1;
    const cv::Vec3f v0 = findInitialVertex( _model, _kdt, spos, f0);
    const cv::Vec3f v1 = findInitialVertex( _model, _kdt, fpos, f1);

    // If the start and end vertices are on the same polygon, the returned list of points is simply the start and end vertex
    if ( f0 == f1)
        _lpath = {v0,v1};
    else
    {
        cv::Vec3f n;
        cv::normalize( _u.cross(v1 - v0), n);
        // If it's not possible to define the global slicing plane orientation (because _u does not make an angle with
        // the path vector), then we can't find the path. _u should always be linearly independent of the path vector.
        if ( l2sq(n) == 0)
            return 0;

        GlobalPlaneSlicingPath sp0( _model, f0, v0, n);
        GlobalPlaneSlicingPath sp1( _model, f1, v1, n);
        _lpath = findBestPath( sp0, sp1);
    }   // end else

    return calcPathLength( _lpath);
}   // end findPath
