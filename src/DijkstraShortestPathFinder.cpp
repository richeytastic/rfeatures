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

#include "DijkstraShortestPathFinder.h"
using RFeatures::DijkstraShortestPathFinder;
using RFeatures::ObjModel;
#include <cassert>
#include <algorithm>
#include <iostream>
#include <sstream>
#include <cmath>
#include <boost/foreach.hpp>
#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>
#include <boost/heap/fibonacci_heap.hpp>

struct Vertex;

// Functor to compare Vertex pointers for max-heap priority queue
struct CompareVertexPathCosts
{
    bool operator()( const Vertex* v0, const Vertex* v1) const;
};  // end struct


typedef boost::heap::fibonacci_heap<Vertex*, boost::heap::compare<CompareVertexPathCosts> >  VertexQueue;
typedef VertexQueue::handle_type PQHandle;


struct Vertex
{
    int uvid;
    cv::Vec3f pos;
    double pathCost;
    const Vertex* prev;   // With category (or integer) weights, there may be several prev vertices
    PQHandle pqhandle;    // Handle to this Vertex for the priority queue

    Vertex( int id, const cv::Vec3f& p=cv::Vec3f(0,0,0), double pcost=0.0, const Vertex* bp=NULL)
        : uvid(id), pos(p), pathCost(pcost), prev(bp)
    { }   // end ctor

    void setHandle( PQHandle h)
    {
        pqhandle = h;
    }   // end setHandle

    // Updates path cost and causes the priority queue to re-heapify
    void updatePathCost( double newPathCost, const Vertex* newPrev)
    {
        pathCost = newPathCost;
        prev = newPrev;
    }   // end updatePathCost
};  // end struct


// Need a min-heap (boost creates a max-heap)
bool CompareVertexPathCosts::operator()( const Vertex* v0, const Vertex* v1) const
{
    return v0->pathCost >= v1->pathCost;
}   // end operator()



// Encapsulates algorithm for Dijkstra's shortest path search.
struct NodeFront
{

// Create a new node front with a starting vertex
NodeFront( const ObjModel::Ptr om, int startUvtx, int finUvtx) : _model(om), _fuvid(finUvtx)
{
    const cv::Vec3f& spos = _model->getVertex( startUvtx);
    _fpos = _model->getVertex( _fuvid);  // Position of the target node
    Vertex* nuv = new Vertex( startUvtx, spos, cv::norm( _fpos - spos), NULL);
    _vtxs[nuv->uvid] = nuv;
    _queue.push(nuv);
}   // end ctor


~NodeFront()
{
    // Delete any vertices remaining on the queue
    VertexQueue::iterator end = _queue.end();
    for ( VertexQueue::iterator it = _queue.begin(); it != end; ++it)
        delete *it;

    typedef std::pair<int, Vertex*> VPair;
    BOOST_FOREACH ( const VPair& v, _expanded)
        delete v.second;
}   // end dtor


// Expand the front until shortest path found to unique vertex _fuvid.
// NULL returned iff no path to the given unique vertex ID could be found,
// otherwise the finish vertex is returned.
const Vertex* expandFront()
{
    const int fuvid = _fuvid;

    // Continue while there are still vertices to explore(expand)
    const Vertex* finishVtx = NULL; // Not NULL once found
    while ( !finishVtx && !_queue.empty())
    {
        const boost::unordered_set<int>* cuvtxs;    // The next vertex's connected vertices
        const Vertex* uv = expandNextVertex( &cuvtxs);

        // Subtract the heuristic cost from the previous point uv for use in calculating expanded node costs
        const double sumPrevPathCost = uv->pathCost - cv::norm( _fpos - uv->pos);

        BOOST_FOREACH ( const int& cid, *cuvtxs)
        {
            // If cid identifies a unique vertex that was already expanded, we can ignore it.
            if ( isExpanded( cid))
                continue;

            const cv::Vec3f& cpos = _model->getVertex(cid);  // Position of this connected vertex
            // Calculate the path sum to this connected vertex from the expanded vertex uv
            double cpathCost = cv::norm( cpos - uv->pos) + sumPrevPathCost;   // Actual costs
            cpathCost += cv::norm( _fpos - cpos); // Add the straight line heuristic

            // If the finish vertex has already been found, we only continue if the newly calculated
            // distance (cpathCost) to this vertex (cid) is not greater than the existing path to finishVtx
            // because if cpathCost is greater, there's no way any path from this vertex (cid) to the
            // finish vertex can ever be any shorter (assuming non-negative path weights of course, which
            // is guaranteed by the fact that we're using Euclidean distance as weight here).
            // A generalised version (with negative weights) would have to deal with this differently.
            if ( finishVtx && cpathCost > finishVtx->pathCost)
                continue;

            // If cid isn't already on the front, add it and check if it's the finish vertex.
            if ( !isOnFront( cid))
            {
                addToSearchFront( cid, cpos, cpathCost, uv);
                if ( cid == fuvid)
                    finishVtx = _vtxs.at(cid);
            }   // end if
            else
            {   // Otherwise, see if this vertex is being reached via a shorter path from uv
                Vertex* vtx = _vtxs.at(cid);
                if ( vtx->pathCost >= cpathCost)
                {
                    vtx->updatePathCost( cpathCost, uv);
                    _queue.increase( vtx->pqhandle);    // O(log(N))
                }   // end if
            }   // end else
        }   // end foreach
    }   // end while

    return finishVtx;
}   // end expandFront


private:
    const ObjModel::Ptr _model;
    const int _fuvid;   // Target vertex ID
    cv::Vec3f _fpos;    // Position of target vertex
    VertexQueue _queue;  // Ordered by distance to Vertex
    boost::unordered_map<int, Vertex*> _vtxs;
    boost::unordered_map<int, Vertex*> _expanded;    // Indices of already expanded unique vertices


    // Get the next best vertex from the search front and place it onto the set of expanded vertices.
    // Also sets the set of connected vertices in output parameter cuvtxs.
    const Vertex* expandNextVertex( const boost::unordered_set<int>** cuvtxs)
    {
        Vertex* uv = _queue.top();
        *cuvtxs = &_model->getConnectedVertices( uv->uvid);
        _queue.pop();
        _vtxs.erase(uv->uvid);
        _expanded[uv->uvid] = uv;
        return uv;
    }   // end expandNextVertex


    bool isOnFront( int uvid) const
    {
        return _vtxs.count(uvid) > 0;
    }   // end isOnFront


    bool isExpanded( int uvid) const
    {
        return _expanded.count(uvid) > 0;
    }   // end isExpanded


    void addToSearchFront( int uvid, const cv::Vec3f& pos, double pathCost, const Vertex* prev)
    {
        Vertex* nuv = new Vertex( uvid, pos, pathCost, prev);
        _vtxs[uvid] = nuv;
        PQHandle h = _queue.push(nuv);   // O(1) for boost::heap::fibonacci_heap
        nuv->setHandle( h);
    }   // end addToSearchFront
};  // end struct



// public
DijkstraShortestPathFinder::DijkstraShortestPathFinder( const ObjModel::Ptr& om) : _model(om)
{}  // end ctor


// public
bool DijkstraShortestPathFinder::setEndPointVertexIndices( int uvA, int uvB)
{
    const IntSet& uvidxs = _model->getVertexIds();
    assert(uvidxs.count(uvA) && uvidxs.count(uvB));
    if ( !uvidxs.count(uvA) || !uvidxs.count(uvB))
        return false;
    _uA = uvA;
    _uB = uvB;
    return true;
}   // end setEndPointVertexIndices


// public
int DijkstraShortestPathFinder::findShortestPath( std::vector<int>& uvids) const
{
    // Check if A and B are the same vertices
    if ( _uA == _uB)
    {
        uvids.push_back(_uA);
        return 0;
    }   // end if

    uvids.clear();

    NodeFront* nfront = new NodeFront( _model, _uA, _uB);
    const Vertex* finVtx = nfront->expandFront();
    if ( finVtx)
    {
        // Copy the shortest path into uvids
        const Vertex* tmp = finVtx;
        while ( tmp)
        {
            uvids.push_back(tmp->uvid);
            tmp = tmp->prev;
        }   // end while
    }   // end if

    delete nfront;

    return (int)uvids.size() - 1;
}   // end findShortestPath
