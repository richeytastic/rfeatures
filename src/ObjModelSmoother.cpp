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

#include <ObjModelSmoother.h>
#include <ObjModelTopologyFinder.h>
#include <boost/heap/fibonacci_heap.hpp>
using RFeatures::ObjModelTopologyFinder;
using RFeatures::ObjModelCurvatureMap;
using RFeatures::ObjModelPolygonAreas;
using RFeatures::ObjModelSmoother;
using RFeatures::ObjModelNormals;
using RFeatures::ObjModel;
using RFeatures::ObjPoly;

namespace {

// Keep a heap of vertices ordered on curvature
struct VertexCurv;
struct VertexCurvComparator { bool operator()( const VertexCurv* v0, const VertexCurv* v1) const; };
typedef boost::heap::fibonacci_heap<VertexCurv*, boost::heap::compare<VertexCurvComparator> > MaxHeap;

struct VertexCurv
{
    VertexCurv( const ObjModelCurvatureMap& c, int vi, const MaxHeap* ph) : cm(c), vidx(vi), pheap(ph) {}

    const ObjModelCurvatureMap& cm;
    int vidx;
    const MaxHeap *pheap; // Pointer to heap for comparing max curvature vertex to this one
    MaxHeap::handle_type handle;

    static double calcCurvature( const ObjModelCurvatureMap& cmap, int vi)
    {
        double kp1;
        cmap.vertexPC1( vi, kp1);
        double kp2;
        cmap.vertexPC2( vi, kp2);
        return (fabs(kp1) + fabs(kp2))/2;
    }   // end calcCurvature

    double curvature() const { return calcCurvature( cm, vidx);}
};  // end struct


bool VertexCurvComparator::operator()( const VertexCurv* v0, const VertexCurv* v1) const
{
    const double c0 = v0->curvature();
    const double c1 = v1->curvature();
    if ( c0 < c1)
        return true;
    if ( c0 > c1)
        return false;

    // In case curvature is equal, prefer vertex that's further from the vertex having maximum curvature.
    const ObjModel* m = v0->cm.model();
    const cv::Vec3f& mpos = m->vtx( v0->pheap->top()->vidx);  // Position of max curvature vertex
    return cv::norm( mpos - m->vtx( v0->vidx)) >= cv::norm( mpos - m->vtx( v1->vidx));
}   // end operator()


int popHeap( MaxHeap& heap, std::unordered_map<int, VertexCurv*>& vcmap, IntSet& hcset)
{
    VertexCurv* vc = heap.top();
    heap.pop();
    const int vidx = vc->vidx;
    hcset.insert(vidx);
    vcmap.erase(vidx);
    delete vc;
    return vidx;
}   // end popHeap


void pushHeap( MaxHeap& heap, std::unordered_map<int, VertexCurv*>& vcmap, int vidx, const ObjModelCurvatureMap& cm)
{
    if ( vcmap.count(vidx) == 0)
    {
        VertexCurv *vc = new VertexCurv( cm, vidx, &heap);
        vc->handle = heap.push(vc);    // O(log(N))
        vcmap[vidx] = vc;
    }   // end if
    else
    {
        VertexCurv *vc = vcmap.at(vidx);
        heap.increase( vc->handle); // O(log(N))
    }   // end else
}   // end pushHeap


// Get all initial vertices having curvature greater than maxc
void createHeap( IntSet& hcset, MaxHeap& heap, std::unordered_map<int, VertexCurv*>& vcmap, const ObjModelCurvatureMap& cm, double maxc)
{
    for ( int vidx : hcset) // for all vertices in the high curvature set
    {
        if ( VertexCurv::calcCurvature( cm, vidx) > maxc)
            pushHeap( heap, vcmap, vidx, cm);
    }   // end for
    hcset.clear();
}   // end createHeap

}   // end namespace


// public
ObjModelSmoother::ObjModelSmoother( ObjModel::Ptr m,
        ObjModelCurvatureMap::Ptr cm,
        ObjModelNormals* nrms,
        ObjModelPolygonAreas* pareas,
        rlib::ProgressDelegate* pd)
    : _model(m), _cmap(cm), _normals(nrms), _pareas(pareas), _progressDelegate(pd)
{
    assert( _model.get() == _cmap->model());
}   // end ctor


// private
void ObjModelSmoother::adjustVertex( int vidx)
{
    cv::Vec3f nv(0,0,0);    // Interpolate new position as mean of connected vertices
    const IntSet& cvidxs = _model->getConnectedVertices(vidx);
    std::for_each( std::begin(cvidxs), std::end(cvidxs), [&](int cv){ nv += _model->vtx(cv);});
    nv = nv * (1.0f/cvidxs.size());
    _model->adjustVertex( vidx, nv);    // Adjust vertex position on model

    const IntSet& fids = _model->getFaceIds( vidx); // Recalculate the polygon areas and normals
    std::for_each( std::begin(fids), std::end(fids), [&](int fid){ _pareas->recalcPolygonArea( fid);});
    std::for_each( std::begin(fids), std::end(fids), [&](int fid){ _normals->recalcFaceNormal( fid);});

    // Finally, recalculate the curvature at this vertex and connected vertices.
    IntSet cvs = _model->getConnectedVertices(vidx);
    cvs.insert(vidx);
    _cmap->map( cvs);
}   // end adjustVertex


// public
void ObjModelSmoother::smooth( double maxc, size_t maxits)
{
    const ObjModelCurvatureMap& cm = *_cmap.get();
    const ObjModel* model = _model.get();

    IntSet hcset;       // Vertices having "high" curvature
    const ObjModelTopologyFinder tfinder( model);
    IntSet boundarySet; // Record vertices that are part of the boundary
    for ( int v : model->getVertexIds())
    {   // Skip boundary vertices and get initial vertices as those only with curvature >= allowed max
        if ( tfinder.isBoundary(v))
            boundarySet.insert(v);
        else if ( VertexCurv::calcCurvature( cm, v) > maxc)
            hcset.insert(v);
    }   // end for

    MaxHeap heap;
    std::unordered_map<int, VertexCurv*> vcmap; // Record mapping of vertex IDs to corresponding curvature node for the MaxHeap

    size_t i = 0;
    while ( i < maxits && !hcset.empty())
    {
        i++;    // Create a max heap of the vertices with maximum curvature which will be a subset of hcset.
        createHeap( hcset, heap, vcmap, cm, maxc);  // hcset empty on return

        while ( !heap.empty())
        {
            int vidx = popHeap( heap, vcmap, hcset);    // vidx inserted into hcset and removed from vcmap.
            adjustVertex(vidx); // High curvature vertex repositioned to be mean of its connected vertices.

            // Parse the connected vertices of the newly adjusted vertex since their curvature will have changed.
            for ( int cv : model->getConnectedVertices(vidx))
            {   // Don't add vertices parsed while heap not empty, don't add boundary vertices, and don't add any with curvature <= maxc
                if ( hcset.count(cv) == 0 && boundarySet.count(cv) == 0 && VertexCurv::calcCurvature( cm, cv) > maxc)
                    pushHeap( heap, vcmap, cv, cm);
            }   // end for
        }   // end while

        if ( _progressDelegate)
            _progressDelegate->updateProgress( float(i)/maxits);
    }   // end while

    assert( heap.empty());
    assert( vcmap.empty());
    if ( _progressDelegate)
         _progressDelegate->updateProgress( 1.0f);
}   // end smooth
