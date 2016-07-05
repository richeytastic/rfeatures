#include "ObjModelTopologyFinder.h"
using RFeatures::ObjModelTopologyFinder;
using RFeatures::ObjModel;
using RFeatures::ObjPoly;
#include <boost/foreach.hpp>


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


// public static
int ObjModelTopologyFinder::getOtherVertex( const ObjPoly& poly, int v0, int v1)
{
    int vn = poly.vindices[0];
    if ( vn == v0 || vn == v1)
    {
        vn = poly.vindices[1];
        if ( vn == v0 || vn == v1)
            vn = poly.vindices[2];
    }   // end if
    return vn;
}   // end getOtherVertex


// public
int ObjModelTopologyFinder::getOtherVertex( int fid, int uvidx0, int uvidx1) const
{
    const ObjPoly& poly =_impl->model->getUniqueVertexFace( fid);
    return getOtherVertex( poly, uvidx0, uvidx1);
}   // end getOtherVertex


// public
int ObjModelTopologyFinder::doesUniquePolyExist( int edgeUvidx0, int edgeUvidx1, int checkUvidx) const
{
    const IntSet& sf = _impl->model->getSharedFaces( edgeUvidx0, edgeUvidx1);
    BOOST_FOREACH ( const int& fid, sf)
    {
        if ( getOtherVertex( fid, edgeUvidx0, edgeUvidx1) == checkUvidx)
            return fid;
    }   // end foreach
    return -1;
}   // end doesUniquePolyExist



// public
ObjModelTopologyFinder::VertexTopology ObjModelTopologyFinder::discoverLocalTopology( int uvid) const
{
    const ObjModel::Ptr model = _impl->model;
    const IntSet& cuvtx = model->getConnectedUniqueVertices( uvid);
    if ( cuvtx.empty())
        return VTX_UNCONNECTED;

    IntSet hset;    // Connected vertices creating edges sharing exactyly one polygon.
    bool flat = true;    // False if connected vertices create edges sharing more than 2 polygons.
    BOOST_FOREACH ( const int& cuv, cuvtx)
    {
        const size_t nshared = model->getNumSharedFaces( uvid, cuv);
        if ( nshared == 0)
        {
            // If it's not connected to any faces then it's part of a 1D edge.
            // Actually can't happen if ObjModel is correctly programmed since this
            // implies no set faces for an existing connection between vertices.
            // This can only happen if removal of a face didn't also correctly detach
            // uvidx from its prior connected vertices for that face.
            // (i.e., there is an integrity error in the ObjModel with face removal).
            assert(false);
            return VTX_UNCONNECTED;
        }   // end if
        else if ( nshared == 1)
            hset.insert(cuv);
        else if ( nshared > 2)
            flat = false;
    }   // end foreach

    int vtopology = VTX_JUNCTION_B;

    // If the local region is "complete" it should be possible to start from a
    // single edge vertex (or an arbitrary vertex if no edge vertices exist)
    // and discover all of the vertices connected to uvid.
    IntSet fcuvtx;  // Found connected vertices
    std::vector<int> xplrNxt;   // Exploration front
    int euv = 0;
    if ( !hset.empty())
    {
        euv = *hset.begin();    // Seed edge vertex
        hset.erase( euv);
        vtopology = VTX_EDGE;
    }   // end if
    else
        euv = *cuvtx.begin();   // Arbitrary connected vertex
    xplrNxt.push_back( euv);
    fcuvtx.insert( euv);        // Discovered connected vertices

    // Set complete to false if need to reseed from a connected edge vertex to find all connected vertices.
    bool complete = true;

    while ( !xplrNxt.empty())
    {
        euv = xplrNxt.back();
        xplrNxt.pop_back();

        // From the shared faces found from edge uvid,euv, add the
        // found connected vertices to xplrNxt.
        const IntSet& fids = model->getSharedFaces( uvid, euv);
        BOOST_FOREACH ( const int& fid, fids)
        {
            const ObjPoly& poly = model->getUniqueVertexFace( fid);
            const int ncuv = getOtherVertex( poly, uvid, euv); // Other vertex on poly not uvid or euv
            if ( !fcuvtx.count(ncuv))
            {
                hset.erase(ncuv);
                xplrNxt.push_back(ncuv);
                fcuvtx.insert(ncuv);
            }   // end if
        }   // end foreach

        // If the exploration front has been exhausted but all connected vertices
        // have not yet been discovered, we check if there are remaining "seed" edge
        // vertices in hset. If there are, we continue with the possibility we can
        // still discover all of the connected vertices and the local topology is
        // "flat". If there are no more elements in hset, the local topology cannot
        // be flat.
        if ( xplrNxt.empty() && fcuvtx.size() < cuvtx.size())
        {
            complete = false;   // Can't complete from a single seed edge

            // If there are no more seed edge vertices, it would be necessary to join
            // one of the seed edge vertices to one of the other (still undiscovered)
            // connected vertices. This would create a pseudo polygon with an edge
            // shared by at least three polygons. If it didn't, then hset wouldn't be
            // empty because the joining vertex would already be considered a seed edge.
            if ( hset.empty())
                flat = false;
            else
            {   // Can continue by looking at other seeds in hset
                euv = *hset.begin();
                hset.erase( euv);
                xplrNxt.push_back( euv);
                fcuvtx.insert( euv);
            }   // end else
        }   // end if
    }   // end while

    if ( complete)
        vtopology |= VTX_COMPLETE;
    if ( flat)
        vtopology |= VTX_FLAT;

    return static_cast<VertexTopology>(vtopology);
}   // end discoverLocalTopology



// public
bool ObjModelTopologyFinder::isBoundary( int uvidx) const
{
    // If there exists a connected vertex to uvidx such that that edge
    // is shared by only a single polygon, then uvidx must be on the boundary.
    const IntSet& cuvtxs = _impl->model->getConnectedUniqueVertices( uvidx);
    bool onBoundary = false;
    BOOST_FOREACH ( const int& cuv, cuvtxs)
    {
        if ( _impl->model->getNumSharedFaces( uvidx, cuv) == 1)
        {
            onBoundary = true;
            break;
        }   // end if
    }   // end foreach

    return onBoundary;
}   // end isBoundary
