/************************************************************************
 * Copyright (C) 2018 Richard Palmer
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

#include <ObjModelSurfacePathFinder.h>
#include <ObjModelSurfacePointFinder.h>
#include <DijkstraShortestPathFinder.h>
#include <FeatureUtils.h>
#include <cassert>
using RFeatures::ObjModelSurfacePathFinder;
using RFeatures::ObjModel;


namespace {

int findClosestVertexIndex( const ObjModel* model, int p, int f, const cv::Vec3f& v)
{
    if ( p >= 0)
        return p;

    assert( f >= 0);
    const int* vidxs = model->fvidxs(f);

    p = vidxs[0];   // vidxs[0] assumed closest
    double d = RFeatures::l2sq( v - model->vtx(p));

    int q = vidxs[1];   // Check against the second vertex
    const double dq = RFeatures::l2sq( v - model->vtx(q));
    if ( dq < d)
    {
        d = dq;
        p = q;
    }   // end if

    q = vidxs[2];   // Check against the third vertex
    if ( RFeatures::l2sq( v - model->vtx(q)) < d)
        p = q;

    return p;
}   // end findClosestVertexIndex

}   // end namespace


double ObjModelSurfacePathFinder::findPath( const ObjModel* model, const cv::Vec3f& spos, const cv::Vec3f& fpos)
{
    _lpath.clear();

    // Find the start and end point vertices
    RFeatures::ObjModelSurfacePointFinder spf( model);
    int f0, f1;
    cv::Vec3f v0, v1;
    int p0 = 0; // First vertex ID.
    spf.find( spos, p0, f0, v0);
    int p1 = p0;
    spf.find( fpos, p1, f1, v1);

    // Ensure that p0 and p1 are valid vertex indices on the model
    p0 = findClosestVertexIndex( model, p0, f0, v0);
    p1 = findClosestVertexIndex( model, p1, f1, v1);

    RFeatures::DijkstraShortestPathFinder dspf( model);
    dspf.setEndPointVertexIndices( p0, p1);
    std::vector<int> pvids;
    dspf.findShortestPath( pvids);
    if ( pvids.size() < 1)
        return -1;

    size_t nps = pvids.size();
    assert( nps >= 2);

    // If the point closest to spos is not incident with a vertex, then
    // check the polygon it's incident with to determine if this is the
    // same polygon as shared by the first two path vertices. If so, then
    // we skip the first discovered path vertex because this leads us
    // away from the desired path direction.
    size_t i = 0;  // Will be the modified start vertex for determined path
    // If the shared face of the first two discovered path vertices == f0, then
    // v0 is incident with face f0 and the first path vertex can be ignored.
    if (( f0 >= 0) && ( model->spolys(pvids[0], pvids[1]).count(f0) > 0))
    {
        _lpath.push_back(v0);   // Initial vertex
        i = 1;
    }   // end if

    // Similarly for the end point
    bool pushv1 = false;
    if (( f1 >= 0) && ( model->spolys(pvids[nps-1], pvids[nps-2]).count(f1) > 0))
    {
        nps--;
        pushv1 = true;
    }   // end if

    cv::Vec3f tmpv = v0;  // For summing over path length
    double psum = 0.0;  // Sum over path length
    for ( ; i < nps; ++i)
    {
        const cv::Vec3f& v = model->vtx( pvids[i]);
        psum += cv::norm( v - tmpv);  // Sum over path length
        tmpv = v;
        _lpath.push_back(v);
    }   // end for

    if ( pushv1)    // If ended early
    {
        _lpath.push_back(v1);   // Last vertex
        psum += cv::norm( v1 - tmpv);
    }   // end if

    return psum;
}   // end findPath
