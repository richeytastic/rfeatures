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

#include <ObjModelSurfacePatches.h>
using RFeatures::ObjModelSurfacePatches;
using RFeatures::ObjModelKDTree;
using RFeatures::ObjModel;
#include <boost/heap/fibonacci_heap.hpp>


namespace {

float sqdistf( const cv::Vec3f& v)
{
    return v[0]*v[0] + v[1]*v[1] + v[2]*v[2];
}   // end sqdistf


class VertexHeap
{
private:
    struct Vtx
    {
        int vid;
        float sqr;   // Squared Euclidean distance from patch centre
    };  // end struct

    struct VtxComparator { bool operator()( const Vtx* v0, const Vtx* v1) const { return v0->sqr > v1->sqr;}};

    typedef boost::heap::fibonacci_heap<Vtx*, boost::heap::compare<VtxComparator> > VtxHeap;

    const ObjModel* _model;
    const cv::Vec3f _pcentre;
    VtxHeap _heap;
    IntSet _vset;

public:
    VertexHeap( const ObjModel* m, const cv::Vec3f& pcentre) : _model(m), _pcentre(pcentre) {}
    ~VertexHeap() { float v; while (!empty()) pop(v); }

    void push( int vid)
    {
        _vset.insert(vid);
        Vtx* v = new Vtx;
        v->vid = vid;
        v->sqr = sqdistf( _pcentre - _model->vtx(vid));
        _heap.push( v); // O(log(N))
    }   // end pushif

    int pop( float& sqr)
    {
        Vtx* v = _heap.top();
        _heap.pop();
        const int vid = v->vid;
        sqr = v->sqr;
        delete v;
        return vid;
    }   // end pop

    bool empty() const { return _heap.empty();}
    bool contains( int vid) const { return _vset.count(vid) > 0;}
};  // end class



int heapFindMaxPoints( const ObjModel* model, int vidx, const cv::Vec3f& centre, float sqR, IntSet& pset, int M)
{
    VertexHeap heap( model, centre);
    heap.push( vidx);    // Start with vertex closest to centre

    float sqr;
    int pcount = 0;
    while ( !heap.empty() && pcount < M)
    {
        vidx = heap.pop( sqr);
        if ( sqr <= sqR)
        {
            pset.insert(vidx);
            pcount++;
            const IntSet& cvs = model->cvtxs( vidx);
            for ( int cv : cvs)
                if ( pset.count(cv) == 0 && !heap.contains(cv)) // Only add vertices not already in the patch or heap
                    heap.push(cv);
        }   // end else
    }   // end while
    return pcount;
}   // end heapFindMaxPoints


int findAllPoints( const ObjModel* model, int vidx, const cv::Vec3f& centre, float sqR, IntSet& sset)
{
    IntSet bset;
    bset.insert(vidx);

    int pcount = 0;
    while ( !bset.empty())
    {
        vidx = *bset.begin();
        bset.erase(vidx);
        if ( sqdistf( centre - model->vtx(vidx)) <= sqR)
        {
            sset.insert(vidx);
            pcount++;
            const IntSet& cvs = model->cvtxs( vidx);
            for ( int cv : cvs)
                if ( sset.count(cv) == 0)
                    bset.insert(cv);
        }   // end if
    }   // end while
    return pcount;
}   // end findAllPoints

}   // end namespace


// public
ObjModelSurfacePatches::ObjModelSurfacePatches( const ObjModelKDTree* t, float R)
    : _dtree(t), _sqR(R*R)
{
}   // end ctor


// public
int ObjModelSurfacePatches::getPatchVertexIds( const cv::Vec3f& v, IntSet& pset, int M) const
{
    const ObjModel* model = _dtree->model();
    int initVid = _dtree->find(v);

    int pcount = 0;
    if ( M > 0)
        pcount = heapFindMaxPoints( model, initVid, v, _sqR, pset, M);
    else if ( M < 0)
        pcount = findAllPoints( model, initVid, v, _sqR, pset);

    return pcount;
}   // end getPatchVertexIds
