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

#include <DijkstraShortestPathFinder.h>
#include <FeatureUtils.h>   // l2sq
using RFeatures::DijkstraShortestPathFinder;
using RFeatures::PathCostCalculator;
using RFeatures::ObjModel;
#include <cassert>
#include <algorithm>
#include <iostream>
#include <sstream>
#include <cmath>
#include <unordered_map>
#include <unordered_set>
#include <boost/heap/fibonacci_heap.hpp>

namespace {

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

    void setHandle( PQHandle h) { pqhandle = h;}

    // Updates path cost and causes the priority queue to re-heapify
    void updatePathCost( double newPathCost, const Vertex* newPrev)
    {
        pathCost = newPathCost;
        prev = newPrev;
    }   // end updatePathCost
};  // end struct


// Need a min-heap (boost creates a max-heap)
bool CompareVertexPathCosts::operator()( const Vertex* v0, const Vertex* v1) const { return v0->pathCost >= v1->pathCost;}


// Encapsulates algorithm for Dijkstra's shortest path search.
struct NodeFront
{

// Create a new node front with a starting vertex
NodeFront( const ObjModel::Ptr om, const PathCostCalculator& pcc, int startUvtx, int finUvtx)
    : _model(om), _pcc(pcc), _fuvid(finUvtx)
{
    const cv::Vec3f& spos = _model->vtx( startUvtx);
    _fpos = _model->vtx( _fuvid);  // Position of the target node
    Vertex* nuv = new Vertex( startUvtx, spos, _pcc( _fpos, spos), NULL);
    _vtxs[nuv->uvid] = nuv;
    _queue.push(nuv);
}   // end ctor


~NodeFront()
{
    // Delete any vertices remaining on the queue
    VertexQueue::iterator end = _queue.end();
    for ( VertexQueue::iterator it = _queue.begin(); it != end; ++it)
        delete *it;
    for ( const auto& v : _expanded)
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
        const std::unordered_set<int>* cuvtxs;    // The next vertex's connected vertices
        const Vertex* uv = expandNextVertex( &cuvtxs);

        // Subtract the heuristic cost from the previous point uv for use in calculating expanded node costs
        const double sumPrevPathCost = uv->pathCost - _pcc( _fpos, uv->pos);

        for ( int cid : *cuvtxs)
        {
            // If cid identifies a unique vertex that was already expanded, we can ignore it.
            if ( isExpanded( cid))
                continue;

            const cv::Vec3f& cpos = _model->vtx(cid);  // Position of this connected vertex
            // Calculate the path sum to this connected vertex from the expanded vertex uv
            double cpathCost = _pcc( cpos, uv->pos) + sumPrevPathCost;   // Actual costs
            cpathCost += _pcc( _fpos, cpos); // Add the A* heuristic to the finish vertex

            // If finish vertex already found, only continue if cpathCost to this vertex (cid) is not greater
            // than existing cost to finishVtx, because if cpathCost is greater, there's no way any path from
            // this vertex (cid) to the finish can ever be any shorter (assuming weights always non-negative).
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
    const PathCostCalculator& _pcc;
    const int _fuvid;   // Target vertex ID
    cv::Vec3f _fpos;    // Position of target vertex
    VertexQueue _queue;  // Ordered by distance to Vertex
    std::unordered_map<int, Vertex*> _vtxs;
    std::unordered_map<int, Vertex*> _expanded;    // Indices of already expanded unique vertices


    // Get the next best vertex from the search front and place it onto the set of expanded vertices.
    // Also sets the set of connected vertices in output parameter cuvtxs.
    const Vertex* expandNextVertex( const std::unordered_set<int>** cuvtxs)
    {
        Vertex* uv = _queue.top();
        *cuvtxs = &_model->getConnectedVertices( uv->uvid);
        _queue.pop();
        _vtxs.erase(uv->uvid);
        _expanded[uv->uvid] = uv;
        return uv;
    }   // end expandNextVertex


    bool isOnFront( int uvid) const { return _vtxs.count(uvid) > 0;}
    bool isExpanded( int uvid) const { return _expanded.count(uvid) > 0;}

    void addToSearchFront( int uvid, const cv::Vec3f& pos, double pathCost, const Vertex* prev)
    {
        Vertex* nuv = new Vertex( uvid, pos, pathCost, prev);
        _vtxs[uvid] = nuv;
        PQHandle h = _queue.push(nuv);   // O(1) for boost::heap::fibonacci_heap
        nuv->setHandle( h);
    }   // end addToSearchFront
};  // end struct

}   // end namespace


// public
double PathCostCalculator::operator()( const cv::Vec3f& v0, const cv::Vec3f& v1) const
{
    return RFeatures::l2sq( v0-v1);
}   // end operator()


// public
DijkstraShortestPathFinder::DijkstraShortestPathFinder( const ObjModel::Ptr om, PathCostCalculator* pcc)
    : _model(om), _pcc(pcc), _delpcc(false), _uA(-1), _uB(-1)
{
    // Use the default (l2-norm) path cost calculator if none provided by client
    if ( !_pcc)
    {
        _pcc = new PathCostCalculator;
        _delpcc = true;
    }   // end if
}  // end ctor


// public
DijkstraShortestPathFinder::~DijkstraShortestPathFinder()
{
    if ( _delpcc)
        delete _pcc;
}   // end dtor


// public
bool DijkstraShortestPathFinder::setEndPointVertexIndices( int uvA, int uvB)
{
    const std::unordered_set<int>& vidxs = _model->getVertexIds();
    assert(vidxs.count(uvA) && vidxs.count(uvB));
    if ( !vidxs.count(uvA) || !vidxs.count(uvB))
        return false;
    _uA = uvA;
    _uB = uvB;
    return true;
}   // end setEndPointVertexIndices


// public
int DijkstraShortestPathFinder::findShortestPath( std::vector<int>& vids, bool clearVector) const
{
    if ( clearVector)
        vids.clear();

    // Check if A and B are the same vertices
    if ( _uA == _uB)
    {
        vids.push_back(_uA);
        return 0;
    }   // end if

    size_t ssize = vids.size();

    NodeFront* nfront = new NodeFront( _model, *_pcc, _uA, _uB);
    const Vertex* finVtx = nfront->expandFront();
    if ( finVtx)
    {
        // Copy the shortest path into vids
        const Vertex* tmp = finVtx;
        while ( tmp)
        {
            // Only add if previous vertex on vids is not tmp-uvid (no duplicates!)
            // Need to do this when the provided vids is not cleared.
            if ( vids.empty() || vids.back() != tmp->uvid)
                vids.push_back(tmp->uvid);
            tmp = tmp->prev;
        }   // end while
    }   // end if

    delete nfront;

    return std::max(0, (int)(vids.size() - ssize) - 1);
}   // end findShortestPath
