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

#include <ObjModelSmoother.h>
#include <boost/heap/fibonacci_heap.hpp>
using RFeatures::ObjModelCurvatureMap;
using RFeatures::ObjModelManifolds;
using RFeatures::ObjModelSmoother;
using RFeatures::ObjModel;
using RFeatures::ObjPoly;

namespace {

// Keep a heap of vertices ordered on curvature
struct VtxCurv;
struct VtxCurvComparator { bool operator()( const VtxCurv* v0, const VtxCurv* v1) const; };
typedef boost::heap::fibonacci_heap<VtxCurv*, boost::heap::compare<VtxCurvComparator> > MaxHeap;

struct VtxCurv
{
    VtxCurv( const ObjModel& m, const ObjModelCurvatureMap& c, int j, int vi, const MaxHeap* ph) : model(m), cm(c), manj(j), vidx(vi), pheap(ph) {}

    const ObjModel& model;
    const ObjModelCurvatureMap& cm;
    int manj;   // Manifold ID
    int vidx;   // Vertex ID
    const MaxHeap *pheap; // Pointer to heap for comparing max curvature vertex to this one
    MaxHeap::handle_type handle;

    static double calcCurvature( const ObjModelCurvatureMap& cmap, int j, int vi)
    {
        double kp1;
        cmap.vertexPC1( j, vi, kp1);
        double kp2;
        cmap.vertexPC2( j, vi, kp2);
        return 0.5 * (fabs(kp1) + fabs(kp2));
    }   // end calcCurvature

    double curvature() const { return calcCurvature( cm, manj, vidx);}
};  // end struct


bool VtxCurvComparator::operator()( const VtxCurv* v0, const VtxCurv* v1) const
{
    const double c0 = v0->curvature();
    const double c1 = v1->curvature();
    if ( c0 < c1)
        return true;
    if ( c0 > c1)
        return false;

    // In case curvature is equal, prefer vertex that's further from the vertex having maximum curvature.
    const ObjModel& m = v0->model;
    assert( !v0->pheap->empty());
    const int vidx = v0->pheap->top()->vidx;
    assert( m.vtxIds().count(vidx) > 0);
    //std::cerr << "VtxCurvComparator::operator(): " << vidx << std::endl;
    const cv::Vec3f& mpos = m.uvtx( vidx);  // Position of max curvature vertex
    return cv::norm( mpos - m.uvtx( v0->vidx)) >= cv::norm( mpos - m.uvtx( v1->vidx));
}   // end operator()


int popHeap( MaxHeap& heap, std::unordered_map<int, VtxCurv*>& vcmap, IntSet& hcset)
{
    VtxCurv* vc = heap.top();
    heap.pop();
    const int vidx = vc->vidx;
    hcset.insert(vidx);
    vcmap.erase(vidx);
    delete vc;
    return vidx;
}   // end popHeap


void pushHeap( MaxHeap& heap, std::unordered_map<int, VtxCurv*>& vcmap, int j, int vidx, const ObjModel& model, const ObjModelCurvatureMap& cm)
{
    if ( vcmap.count(vidx) == 0)
    {
        VtxCurv *vc = new VtxCurv( model, cm, j, vidx, &heap);
        vc->handle = heap.push(vc);    // O(log(N))
        vcmap[vidx] = vc;
    }   // end if
    else
    {
        VtxCurv *vc = vcmap.at(vidx);
        heap.increase( vc->handle); // O(log(N))
    }   // end else
}   // end pushHeap


// Get all initial vertices having curvature greater than maxc
void createHeap( IntSet& hcset,
                 MaxHeap& heap,
                 std::unordered_map<int, VtxCurv*>& vcmap,
                 const ObjModel& model, const ObjModelCurvatureMap& cm, int manj, double maxc)
{
    for ( int vidx : hcset) // for all vertices in the high curvature set
    {
        if ( VtxCurv::calcCurvature( cm, manj, vidx) > maxc)
            pushHeap( heap, vcmap, manj, vidx, model, cm);
    }   // end for
    hcset.clear();
}   // end createHeap

}   // end namespace


// public
ObjModelSmoother::ObjModelSmoother( ObjModel& m, ObjModelCurvatureMap& cmap, const ObjModelManifolds& manf)
    : _model(m), _cmap(cmap), _manf(manf) {}


// private
void ObjModelSmoother::_adjustVertex( int manj, int vidx)
{
    const IntSet& mvids = _manf.manifold(manj)->vertices(_model);
    cv::Vec3f nv(0,0,0);    // Interpolate new position as mean of connected vertices
    const IntSet& cvtxs = _model.cvtxs(vidx);
    int cnt = 0;
    for ( int cv : cvtxs)
    {
        if ( mvids.count(cv) > 0)  // Only count connected vertices within the manifold
        {
            nv += _model.uvtx(cv);
            cnt++;
        }   // end if
    }   // end for
    nv = nv * 1.0f/cnt;
    _model.adjustVertex( vidx, nv);    // Adjust vertex position on model
}   // end _adjustVertex


// public
void ObjModelSmoother::smooth( double maxc, size_t maxits)
{
    const int nm = static_cast<int>(_manf.count());

    for ( int manj = 0; manj < nm; ++manj)  // For each manifold
    {
        IntSet boundarySet; // Record vertices that are part of the boundary
        for ( int eid : _manf.manifold(manj)->edges())
        {
            const auto& edge = _model.edge(eid);
            boundarySet.insert( edge[0]);
            boundarySet.insert( edge[1]);
        }   // end for

        IntSet hcset;       // Vertices with curvature more than the allowed max
        for ( int vid : _manf.manifold(manj)->vertices(_model))
        {   // Skip boundary vertices and get initial vertices as those only with curvature > allowed max
            if ( boundarySet.count(vid) == 0 && VtxCurv::calcCurvature( _cmap, manj, vid) > maxc)
                hcset.insert(vid);
        }   // end for

        MaxHeap heap;
        std::unordered_map<int, VtxCurv*> vcmap; // Map vertex IDs to corresponding curvature MaxHeap nodes

        const IntSet& mvids = _manf.manifold(manj)->vertices(_model);
        size_t i = 0;
        while ( i < maxits && !hcset.empty())
        {
            i++;    // Create a max heap of the vertices with maximum curvature which will be a subset of hcset.
            createHeap( hcset, heap, vcmap, _model, _cmap, manj, maxc);  // hcset empty on return

            while ( !heap.empty())
            {
                const int vidx = popHeap( heap, vcmap, hcset);    // vidx inserted into hcset and removed from vcmap.
                _adjustVertex( manj, vidx); // High curvature vertex repositioned to be mean of its connected vertices.
                _cmap.update( _model, _manf, vidx);      // Update curvature at the vertex (and vertices connected to it).

                // Parse the connected vertices of the newly adjusted vertex since their curvature will have changed.
                for ( int cv : _model.cvtxs(vidx))
                {
                    if ( mvids.count(cv) == 0)  // Don't consider vertices not on the manifold
                        continue;

                    // Don't add vertices parsed while heap not empty, don't add boundary vertices, and don't add any with curvature <= maxc
                    if ( hcset.count(cv) == 0 && boundarySet.count(cv) == 0 && VtxCurv::calcCurvature( _cmap, manj, cv) > maxc)
                        pushHeap( heap, vcmap, manj, cv, _model, _cmap);
                }   // end for
            }   // end while
        }   // end while

        assert( heap.empty());
        assert( vcmap.empty());
    }   // end for
}   // end smooth
