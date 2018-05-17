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

#ifndef RFEATURES_OBJ_MODEL_VERTEX_CROSSING_TIME_CALCULATOR_H
#define RFEATURES_OBJ_MODEL_VERTEX_CROSSING_TIME_CALCULATOR_H

/**
 * Calculate crossing times of a front moving over the vertices of a triangulated manifold
 * using methods from "Computing geodesic paths on manifolds" by R.Kimmel and J.A.Sethian (1998).
 */

#include "ObjModelPolygonAngles.h"

namespace RFeatures {

class rFeatures_EXPORT ObjModelVertexCrossingTimeCalculator
{
public:
    ObjModelVertexCrossingTimeCalculator( const ObjModel*,
                                          std::unordered_map<int, double> &times,  // crossings - input->time (from a single source)
                                          FaceAngles *faceAngles=NULL);            // If not provided, will be calculated on the fly

    // The uvidx crossing times allow for times from multiple source locations.
    // Parameter srcID must indicate which of the sources to use.
    ObjModelVertexCrossingTimeCalculator( const ObjModel*,
                                          std::unordered_map<int, std::unordered_map<int, double> > &times,  // crossings - input->source->time
                                          int srcID,                    // Must be present in the times value map.
                                          FaceAngles *faceAngles=NULL); // If not provided, will be calculated on the fly

    virtual ~ObjModelVertexCrossingTimeCalculator();

    // Calculate the crossing time at uvidx C with front going at speed F.
    // If C is not present in the provided vertex crossings map, it is initialised with time DBL_MAX.
    double operator()( int C, double F);

    // tB: time of front crossing at B
    // tA: time of front crossing at A
    // a: length of triangle side opposite vertex A
    // b: length of triangle side opposite vertex B
    // thetaAtC: angle (radians) inside triangle at vertex C
    // F: propagation rate at vertex C (given as time/distance)
    static double calcTimeAtC( double tB, double tA, double a, double b, double thetaAtC, double F);

private:
    const ObjModel* _model;
    FaceAngles *_faceAngles;
    struct VCrossingTimes;
    VCrossingTimes *_vtimes;
};  // end class

}   // end namespace

#endif
