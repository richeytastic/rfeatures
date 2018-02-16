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

#include <ObjModelTopologyFinder.h>
using RFeatures::ObjModelTopologyFinder;
using RFeatures::ObjModel;
using RFeatures::ObjPoly;
#include <boost/foreach.hpp>
#include <stack>


struct ObjModelTopologyFinder::Impl
{
    ObjModel::Ptr model;
};  // end struct


// public
ObjModelTopologyFinder::ObjModelTopologyFinder( const ObjModel::Ptr& om) : _impl(new Impl)
{
    _impl->model = om;
}   // end ctor


// public
ObjModelTopologyFinder::~ObjModelTopologyFinder()
{
    delete _impl;
}   // end dtor


// public
int ObjModelTopologyFinder::doesPolyExist( int edgeUvidx0, int edgeUvidx1, int checkUvidx) const
{
    const IntSet& sf = _impl->model->getSharedFaces( edgeUvidx0, edgeUvidx1);
    BOOST_FOREACH ( int fid, sf)
    {
        const ObjPoly& face = _impl->model->getFace( fid);
        if ( face.getOpposite( edgeUvidx0, edgeUvidx1) == checkUvidx)
            return fid;
    }   // end foreach
    return -1;
}   // end doesPolyExist


// public
ObjModelTopologyFinder::BasicTopology ObjModelTopologyFinder::getBasicTopology( int uvid) const
{
    const ObjModel::Ptr model = _impl->model;
    const IntSet& cuvtx = model->getConnectedVertices( uvid);
    const IntSet& fids = model->getFaceIds( uvid);
    if ( cuvtx.empty())
        return VTX_UNCONNECTED;
    else if ( cuvtx.size() >= 1 && fids.empty())
        return VTX_LINE;
    else if ( fids.size() == 1)
        return VTX_TIP;
    return VTX_COMPLEX;
}   // end getBasicTopology


// public
ObjModelTopologyFinder::ComplexTopology ObjModelTopologyFinder::getComplexTopology( int uvid) const
{
    const ObjModel::Ptr model = _impl->model;
    const IntSet& cuvtx = model->getConnectedVertices( uvid);
    assert( !cuvtx.empty());

    IntSet hset;        // Connected vertices creating edges sharing exactly one polygon.
    bool flat = true;   // False if connected vertices create edges sharing more than 2 polygons.
    BOOST_FOREACH ( int cuv, cuvtx)
    {
        const size_t nshared = model->getNumSharedFaces( uvid, cuv);
        assert( nshared > 0);
        if ( nshared == 1)
            hset.insert(cuv);
        else if ( nshared > 2)
            flat = false;
    }   // end foreach

    int vtopology = VTX_JUNCTION_B;

    // If the local region is "complete" it should be possible to start from a single edge vertex (or an
    // arbitrary vertex if no edge vertices exist) and discover all of the vertices connected to uvid.
    IntSet fcuvtx;             // Found connected vertices
    std::stack<int> xplrNxt;   // Exploration front
    int euv = 0;
    if ( !hset.empty())
    {
        euv = *hset.begin();    // Seed edge vertex
        hset.erase( euv);
        vtopology = VTX_EDGE;
    }   // end if
    else
        euv = *cuvtx.begin();   // Arbitrary connected vertex (should be possible to get to others from here)
    xplrNxt.push( euv);
    fcuvtx.insert( euv);        // Discovered connected vertices

    // Set complete to false if need to reseed from a connected edge vertex to find all connected vertices.
    bool complete = true;

    while ( !xplrNxt.empty())
    {
        euv = xplrNxt.top();
        xplrNxt.pop();

        // From the shared faces found from edge uvid,euv, add the found connected vertices to xplrNxt.
        const IntSet& fids = model->getSharedFaces( uvid, euv);
        BOOST_FOREACH ( int fid, fids)
        {
            const ObjPoly& poly = model->getFace( fid);
            const int ncuv = poly.getOpposite( uvid, euv); // Other vertex on poly not uvid or euv
            if ( !fcuvtx.count(ncuv))
            {
                hset.erase(ncuv);
                xplrNxt.push(ncuv);
                fcuvtx.insert(ncuv);
            }   // end if
        }   // end foreach

        // If the exploration front has been exhausted but all connected vertices
        // have not yet been discovered, we check if there are remaining "seed" edge
        // vertices in hset. If there are, we continue with the possibility we can
        // still discover all of the connected vertices and the local topology is "flat".
        // If there were never any single poly edges, the local topology cannot be flat.
        if ( xplrNxt.empty() && fcuvtx.size() < cuvtx.size())
        {
            complete = false;   // Can't complete from a single seed edge

            // If there were never any single poly edges in the first place, it is not possible
            // to traverse between two surfaces except through uvid and uvid is JUNCTION_B (VTX_EDGE not set).
            // But if there were single poly edges originally, but they're now exhausted, then
            // we've just traversed a polygonal shared connected to a surface at uvid so this is
            // a JUNCTION_A (since VTX_EDGE is set).
            if ( hset.empty())
                flat = false;
            // If we can continue to try to find the remaining connected vertices because
            // there are other single poly edge vertices, then we keep going.
            else if ( !hset.empty())
            {   // Can continue by looking at other seeds in hset
                euv = *hset.begin();
                hset.erase( euv);
                xplrNxt.push( euv);
                fcuvtx.insert( euv);
            }   // end else
        }   // end if
    }   // end while

    if ( complete)
        vtopology |= VTX_COMPLETE;
    if ( flat)
        vtopology |= VTX_FLAT;

    return static_cast<ComplexTopology>(vtopology);
}   // end getComplexTopology


// public
bool ObjModelTopologyFinder::isBoundary( int vidx) const
{
    // vidx is on a boundary if there exists a second vertex vidx1 where edge vidx-->vidx1 is shared by just a single polygon.
    const IntSet& cvtxs = _impl->model->getConnectedVertices( vidx);
    BOOST_FOREACH ( int cv, cvtxs)
        if ( _impl->model->getNumSharedFaces( vidx, cv) == 1)
            return true;
    return false;
}   // end isBoundary
