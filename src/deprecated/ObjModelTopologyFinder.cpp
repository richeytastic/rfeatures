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
#include <stack>


// public
ObjModelTopologyFinder::ObjModelTopologyFinder( const ObjModel* m) : _model(m)
{
}   // end ctor


// public
int ObjModelTopologyFinder::doesPolyExist( int edgeVidx0, int edgeVidx1, int checkVidx) const
{
    const IntSet& sf = _model->getSharedFaces( edgeVidx0, edgeVidx1);
    for ( int fid : sf)
    {
        const ObjPoly& face = _model->getFace( fid);
        if ( face.opposite( edgeVidx0, edgeVidx1) == checkVidx)
            return fid;
    }   // end foreach
    return -1;
}   // end doesPolyExist


// public
ObjModelTopologyFinder::BasicTopology ObjModelTopologyFinder::getBasicTopology( int vid) const
{
    const IntSet& cvtxs = _model->cvtxs( vid);
    const IntSet& fids = _model->getFaceIds( vid);
    if ( cvtxs.empty())
        return VTX_UNCONNECTED;
    else if ( cvtxs.size() >= 1 && fids.empty())
        return VTX_LINE;
    else if ( fids.size() == 1)
        return VTX_TIP;
    return VTX_COMPLEX;
}   // end getBasicTopology


// public
ObjModelTopologyFinder::ComplexTopology ObjModelTopologyFinder::getComplexTopology( int vid) const
{
    const IntSet& cvtxs = _model->cvtxs( vid);
    assert( !cvtxs.empty());

    IntSet hset;        // Connected vertices of edges sharing exactly one polygon.
    bool flat = true;   // False if connected vertices create edges sharing more than 2 polygons.
    for ( int cv : cvtxs)
    {
        const size_t nshared = _model->getNumSharedFaces( vid, cv);
        assert( nshared > 0);
        if ( nshared == 1)
            hset.insert(cv);
        else if ( nshared > 2)
            flat = false;
    }   // end foreach

    int vtopology = VTX_JUNCTION_B;

    // If the local region is "complete" it should be possible to start from a single edge vertex (or an
    // arbitrary vertex if no edge vertices exist) and discover all of the vertices connected to vid.
    IntSet fcvtxs;             // Found connected vertices
    std::stack<int> xplrNxt;   // Exploration front
    int euv = 0;
    if ( !hset.empty())
    {
        euv = *hset.begin();    // Seed edge vertex
        hset.erase( euv);
        vtopology = VTX_EDGE;
    }   // end if
    else
        euv = *cvtxs.begin();   // Arbitrary connected vertex (should be possible to get to others from here)
    xplrNxt.push( euv);
    fcvtxs.insert( euv);        // Discovered connected vertices

    // Set complete to false if need to reseed from a connected edge vertex to find all connected vertices.
    bool complete = true;

    while ( !xplrNxt.empty())
    {
        euv = xplrNxt.top();
        xplrNxt.pop();

        // From the shared faces found from edge vid,euv, add the found connected vertices to xplrNxt.
        const IntSet& fids = _model->getSharedFaces( vid, euv);
        for ( int fid : fids)
        {
            const ObjPoly& poly = _model->getFace( fid);
            const int ncv = poly.getOpposite( vid, euv); // Other vertex on poly not vid or euv
            if ( !fcvtxs.count(ncv))
            {
                hset.erase(ncv);
                xplrNxt.push(ncv);
                fcvtxs.insert(ncv);
            }   // end if
        }   // end foreach

        // If the exploration front has been exhausted but all connected vertices
        // have not yet been discovered, we check if there are remaining "seed" edge
        // vertices in hset. If there are, we continue with the possibility we can
        // still discover all of the connected vertices and the local topology is "flat".
        // If there were never any single poly edges, the local topology cannot be flat.
        if ( xplrNxt.empty() && fcvtxs.size() < cvtxs.size())
        {
            complete = false;   // Can't complete from a single seed edge

            // If there were never any single poly edges in the first place, it is not possible
            // to traverse between two surfaces except through vid and vid is JUNCTION_B (VTX_EDGE not set).
            // But if there were single poly edges originally, but they're now exhausted, then
            // we've just traversed a polygonal shared connected to a surface at vid so this is
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
                fcvtxs.insert( euv);
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
    for ( int cv : _model->cvtxs( vidx))
        if ( _model->getNumSharedFaces( vidx, cv) == 1)
            return true;
    return false;
}   // end isBoundary
