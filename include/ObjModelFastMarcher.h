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

#ifndef RFEATURES_OBJ_MODEL_FAST_MARCHER_H
#define RFEATURES_OBJ_MODEL_FAST_MARCHER_H

/**
 * Implements Sethian's Fast Marching algorithm on a triangulated manifold.
 * Runs in ~ O(N log(N))
 */

#include <ObjModel.h>
#include <ObjModelFaceAngleCalculator.h>
#include <boost/heap/fibonacci_heap.hpp>

namespace RFeatures {

struct MinVertex;
struct MaxVertex;
struct VertexMinTimeComparator { bool operator()( const MinVertex* v0, const MinVertex* v1) const;};
struct VertexMaxTimeComparator { bool operator()( const MaxVertex* v0, const MaxVertex* v1) const;};
typedef boost::heap::fibonacci_heap<MinVertex*, boost::heap::compare<VertexMinTimeComparator> > MinHeap;
typedef boost::heap::fibonacci_heap<MaxVertex*, boost::heap::compare<VertexMaxTimeComparator> > MaxHeap;

struct MinVertex
{
    MinVertex( int ui, double t) : vidx(ui), time(t) { }   // end ctor
    int vidx;        // The vertex on the model that this MinVertex is closest to.
    double time;     // The time at which this vertex is crossed
    MinHeap::handle_type minHeapHandle;
};  // end struct

struct MaxVertex
{
    MaxVertex( int ui, double t) : vidx(ui), time(t) { }   // end ctor
    int vidx;        // The vertex on the model that this MinVertex is closest to.
    double time;     // The time at which this vertex is crossed
    MaxHeap::handle_type maxHeapHandle;
};  // end struct



class rFeatures_EXPORT ObjModelFastMarcher
{
public:
    // The basic speed function may be designed to respond to the surface function of the
    // underlying model (e.g. curvature at a vertex). By default, a uniform speed is used.
    struct SpeedFunctor { virtual double operator()( int vidx) const { return 1.0;} };

    // If FaceAngles is NULL, they will be recalculated on the fly (less efficient).
    typedef boost::shared_ptr<ObjModelFastMarcher> Ptr;
    static Ptr create( const ObjModel::Ptr m, const SpeedFunctor*, FaceAngles* fa=NULL);
    ObjModelFastMarcher( const ObjModel::Ptr m, const SpeedFunctor*, FaceAngles* fa=NULL);
    virtual ~ObjModelFastMarcher();

    const ObjModel::Ptr getModel() const { return _model;}  // Get the input model

    // Update the vertex time crossing map using a fast marching front
    // that propagates outwards from the starting vertex (initial entry on _minHeap).
    // On return, provided tmap will have been updated only with the vertices that
    // can be reached from vidx. Returns the ID of the unique vertex reached
    // last by the propagating front which will be the most distant from vidx.
    // t is the initial time of arrival at vertex vidx. Typically 0 unless making
    // multiple sequentially updating calls to the time map.
    int propagateFront( int vidx, double t=0.0);

    // Return the vertex time crossing map. Only vertices reachable from vidx (passed
    // into propagateFront) are in this map. Subsequent calls to propagateFront will
    // update this mapping.
    const std::unordered_map<int, double>& getCrossings() const { return _time;}

    void reset();   // Reset (clear) the time crossing maps

protected:
    virtual int expandFront();

    bool withinNarrowBand(int C) const;   // True if C is within the narrow propagation front
    bool isFixed(int C) const;            // True if C's arrival time is fixed
    double getInverseSpeedAt(int C) const;// Get the inverse speed F (time/distance) at C

    int popVertex();
    void addVertex( int ui, double t);
    void updateVertex( int ui, double t);

    std::unordered_map<int, double>& getEditableCrossings() { return _time;}

private:
    const ObjModel::Ptr _model;
    const SpeedFunctor* _speedFunctor;
    FaceAngles *_faceAngles;
    bool _delfa;
    std::unordered_map<int, double> _time;          // Time when front passed vertices

    std::unordered_map<int, MinVertex*> *_narrowBand;  // narrow band of vertices constituting the propagating front
    std::unordered_set<int> *_fixed;                   // members of _time with fixed times
    MinHeap *_minHeap;           // Priority queue of _narrowBand allows for O(1) access to vertex closest to fixed
};  // end class

}   // end namespace

#endif
