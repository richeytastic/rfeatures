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

#ifndef RFEATURES_OBJ_MODEL_REMESHER_H
#define RFEATURES_OBJ_MODEL_REMESHER_H

/**
 * Remeshes a triangulated mesh from a vertex furthest from a given starting vertex.
 * Implements methods from "Geodesic Remeshing Using Front Propagation" by Peyre and Cohen (2006).
 */

#include "ObjPolyInterpolator.h"

namespace RFeatures {

class rFeatures_EXPORT ObjModelRemesher
{
public:
    // If no speed function provided, the default uniform speed function will be used.
    // If FaceAngles is NULL, they will be recalculated on the input model as needed and cached.
    ObjModelRemesher( const ObjModel*, const ObjModelFastMarcher::SpeedFunctor*, FaceAngles *fa=NULL);
    virtual ~ObjModelRemesher();

    // Sample the model with n points from the given starting vertex ID.
    // The sampled model will use only the available input vertices.
    // If the input model does not have a sufficient density of points, this can
    // lead to poor retriangulation (triangles on average not being very equalateral).
    // In this case, use sampleInterpolated().
    // Returns actual number of vertices sampled or < 0 on error.
    int sample( int startVidx, int n);

    // In this version, new vertices will be created by interpolating over the triangles of the input model.
    // Returns actual number of vertices sampled or < 0 on error.
    int sampleInterpolated( int startVidx, int n);

    ObjModel::Ptr getSampleObject() const { return _outmod;}

    // After calling one of the sample functions, call this to identify sampled pairs of vertices that create edges.
    // These can then be used (after adding the required vertices to a model) to add polygons using ObjModelEdgeFaceAdder.
    void createSaddleEdges( std::unordered_map<int,IntSet>&) const;

private:
    const ObjModel* _inmod;
    const ObjModelFastMarcher::SpeedFunctor *_speedFunctor;
    FaceAngles *_faceAngles;
    bool _delfa;

    std::unordered_map<int, std::unordered_map<int, double> > _vTimes; // Input vertices mapped to output source vidxs crossing times
    std::unordered_map<int, int> _nearestSources;                      // Input vertices mapped to nearest output source vidxs
    IntSet _saddlePoints;                             // vidxs of the input vertices that are saddle points

    ObjModel::Ptr _outmod;

    MinHeap _iminHeap;                                // Priority queue of input vertices with closest to current source at top.
    std::unordered_map<int, MinVertex*> _inarrowBand; // Narrow band of updating vertex crossing times using current source.
    IntSet _ifixedSet;                                // Fixed input vertex crossing times against current source for one iteration.

    MaxHeap _maxHeap;                                 // For keeping track of maximal times to vertices (input model)
    std::unordered_map<int, MaxVertex*> _heapMap;     // Track which vertices are in _maxHeap

    void expandInputFront( int);
    void setVertexNearestSource( int, int);
    double calcTimeFromSource( int, const cv::Vec3f&) const;

    int popNarrowBand();
    void updateNarrowBand( int, double);
    int popMaxDistanceHeap( double&);
    void updateMaxHeap( int, double);
    void resetMaxDistanceHeap();
    void init();

    int sample( int, int, bool);
};  // end class

}   // end namespace

#endif
