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
using RFeatures::ObjModelCurvatureMap;
using RFeatures::ObjModelSmoother;
using RFeatures::ObjModel;
using RFeatures::ObjPoly;
#include <boost/foreach.hpp>
#include <boost/heap/fibonacci_heap.hpp>

// Keep a heap of vertices ordered on curvature
struct VertexCurv;
struct VertexCurvComparator { bool operator()( const VertexCurv* v0, const VertexCurv* v1) const; };
typedef boost::heap::fibonacci_heap<VertexCurv*, boost::heap::compare<VertexCurvComparator> > MaxHeap;

struct VertexCurv
{
    VertexCurv( int vi, ObjModelCurvatureMap::Ptr c, const MaxHeap* ph) : vidx(vi), cm(c), pheap(ph) {}
    int vidx;
    ObjModelCurvatureMap::Ptr cm;
    const MaxHeap *pheap; // Pointer to heap for comparing max curvature vertex to this one
    MaxHeap::handle_type handle;

    static double getCurvature( int vi, ObjModelCurvatureMap::Ptr cmap)
    {
        double kp1;
        cmap->getVertexPrincipalCurvature1( vi, kp1);
        //return fabs(kp1);
        double kp2;
        cmap->getVertexPrincipalCurvature2( vi, kp2);
        return (fabs(kp1) + fabs(kp2))/2;
    }   // end getCurvature

    double getCurvature() const { return getCurvature( vidx, cm);}
};  // end struct


bool VertexCurvComparator::operator()( const VertexCurv* v0, const VertexCurv* v1) const
{
    const double c0 = v0->getCurvature();
    const double c1 = v1->getCurvature();
    if ( c0 < c1)
        return true;
    if ( c0 > c1)
        return false;

    // In the case that curvature is equal, prefer the vertex that's further from the vertex having maximum curvature.
    const ObjModel::Ptr model = v0->cm->getObject();
    const cv::Vec3f& mpos = model->getVertex( v0->pheap->top()->vidx);  // Position of max curvature vertex
    return cv::norm( mpos - model->getVertex( v0->vidx)) >= cv::norm( mpos - model->getVertex( v1->vidx));
}   // end operator()


int popHeap( MaxHeap& heap, boost::unordered_map<int, VertexCurv*>& vcmap)
{
    VertexCurv* vc = heap.top();
    heap.pop();
    const int vidx = vc->vidx;
    vcmap.erase(vidx);
    delete vc;
    return vidx;
}   // end popHeap


void pushHeap( MaxHeap& heap, boost::unordered_map<int, VertexCurv*>& vcmap, int vidx, ObjModelCurvatureMap::Ptr cm)
{
    if ( !vcmap.count(vidx))
    {
        VertexCurv *vc = new VertexCurv( vidx, cm, &heap);
        vc->handle = heap.push(vc);    // O(log(N))
        vcmap[vidx] = vc;
    }   // end if
    else
    {
        VertexCurv *vc = vcmap.at(vidx);
        heap.increase( vc->handle); // O(log(N))
    }   // end else
}   // end pushHeap


void createInitialHighCurvSet( IntSet& hcset, const ObjModelCurvatureMap::Ptr cm, double maxc, const IntSet& bverts)
{
    const IntSet& vidxs = cm->getObject()->getVertexIds();
    BOOST_FOREACH ( const int& vidx, vidxs)
    {
        if ( bverts.count(vidx))
            continue;

        if ( VertexCurv::getCurvature( vidx, cm) >= maxc)
            hcset.insert(vidx);
    }   // end foreach
}   // end createInitialHighCurvSet


// Get all initial vertices having curvature greater than maxc
void createHeap( IntSet& hcset, MaxHeap& heap, boost::unordered_map<int, VertexCurv*>& vcmap, ObjModelCurvatureMap::Ptr cm, double maxc)
{
    BOOST_FOREACH ( const int& vidx, hcset)
    {
        if ( VertexCurv::getCurvature( vidx, cm) >= maxc)
            pushHeap( heap, vcmap, vidx, cm);
    }   // end foreach
}   // end createHeap


// public
ObjModelSmoother::ObjModelSmoother( ObjModelCurvatureMap::Ptr cm)
    : _curvMap(cm), _progressDelegate(NULL)
{
    assert( _curvMap != NULL);
}   // end ctor


// public
void ObjModelSmoother::setProgressDelegate( rlib::ProgressDelegate* pd)
{
    _progressDelegate = pd;
}   // end setProgressDelegate


cv::Vec3d interpolateOverConnected( const ObjModel::Ptr model, int vidx)
{
    cv::Vec3d v(0,0,0);
    const IntSet& cvidxs = model->getConnectedVertices(vidx);
    BOOST_FOREACH ( const int& cv, cvidxs)
        v += model->getVertex(cv);
    return v * (1./int(cvidxs.size()));
}   // end interpolateOverConnected


void getBoundaryVertices( const ObjModel::Ptr model, IntSet& bverts)
{
    RFeatures::ObjModelTopologyFinder tfinder( model);
    const IntSet& vidxs = model->getVertexIds();
    BOOST_FOREACH ( int vidx, vidxs)
    {
        if ( tfinder.isBoundary(vidx))
            bverts.insert(vidx);
    }   // end foreach
}   // end getBoundaryVertices



// public
double ObjModelSmoother::smooth( double maxc, size_t& numIterations, bool includeBoundary)
{
    int vidx;
    double curv;
    cv::Vec3f nvtx;
    ObjModel::Ptr model = _curvMap->getObject();

    IntSet bverts;
    if ( !includeBoundary)
        getBoundaryVertices( model, bverts);

    IntSet poppedSet;
    createInitialHighCurvSet( poppedSet, _curvMap, maxc, bverts);

    MaxHeap heap;
    boost::unordered_map<int, VertexCurv*> vcmap;

    size_t i = 0;
    while ( i < numIterations && !poppedSet.empty())
    {
        i++;
        createHeap( poppedSet, heap, vcmap, _curvMap, maxc);
        poppedSet.clear();

        while ( !heap.empty())
        {
            vidx = popHeap( heap, vcmap);
            poppedSet.insert(vidx);

            curv = VertexCurv::getCurvature( vidx, _curvMap);

            nvtx = interpolateOverConnected( model, vidx);
            model->adjustVertex( vidx, cv::Vec3f( nvtx[0], nvtx[1], nvtx[2]));
            _curvMap->recalcVertex( vidx);

            const IntSet& cvidxs = model->getConnectedVertices(vidx);
            BOOST_FOREACH ( const int& cv, cvidxs)
            {
                // Don't add boundary vertices or vertices parsed within this iteration
                if ( bverts.count(cv) || poppedSet.count(cv))
                    continue;

                if ( VertexCurv::getCurvature( cv, _curvMap) > maxc)
                    pushHeap( heap, vcmap, cv, _curvMap);
            }   // end foreach
        }   // end while

        if ( _progressDelegate)
            _progressDelegate->updateProgress( float(i)/numIterations);
    }   // end while

    if ( _progressDelegate && i < numIterations) // Iterations may end earlier than expected if poppedSet.empty().
        _progressDelegate->updateProgress( 1.0f);

    numIterations = i;

    assert( heap.empty());
    assert( vcmap.empty());

    // Get the max curvature from the address vertices.
    double lastMaxCurv = maxc;
    BOOST_FOREACH ( int vidx, poppedSet)
        lastMaxCurv = std::max( VertexCurv::getCurvature( vidx, _curvMap), lastMaxCurv);
    poppedSet.clear();
    return lastMaxCurv;
}   // end smooth
