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

#ifndef RFEATURES_OBJ_MODEL_FACE_UNFOLDING_VERTEX_SEARCHER_H
#define RFEATURES_OBJ_MODEL_FACE_UNFOLDING_VERTEX_SEARCHER_H

/**
 * Helper algorithm for the Fast Marching front propagation algorithm
 * used in class ObjModelVertexCrossingTimeCalculator.
 */

#include "ObjModel.h"

namespace RFeatures {

class ObjModelPolyUnfolder;

class rFeatures_EXPORT ObjModelFaceUnfoldingVertexSearcher
{
public:
    explicit ObjModelFaceUnfoldingVertexSearcher( const ObjModel*);

    const ObjModel* model() const { return _model;}

    // Given poly T on the model having vertex vi, recursively unfold adjacent triangles
    // (starting on the opposite edge to vi) to lie in the plane defined by triangle T,
    // until a vertex on an unfolded triangle is within an acute section having angle
    // PI - theta, where theta is the inner angle at vi on poly T.
    // Returns the vertex ID of the identified vertex, and sets unfoldedPos to the
    // position of this vertex in the unfolded plane.
    // -1 is returned if a suitable vertex cannot be found.
    int operator()( int vi, int T, double theta, cv::Vec3f& unfoldedPos);
    int operator()( int vi, int T, cv::Vec3f& unfoldedPos); // As above but theta is calculated

private:
	const ObjModel* _model;

    cv::Vec3d _cVec;           // The position vector of the initial corner (vi)
    cv::Vec3d _secDirVec;      // Unit length direction vector for the search section
    cv::Vec3d _initEdgePos;    // Position of the root of the first unfolding edge vector.
    cv::Vec3d _initUnitEdge;   // The first unfolding edge vector (unit).
    double _halfInitEdgeNorm;  // Half length of initial edge
    double _alpha;             // Search section angle centred on _secDirVec.

    std::unordered_set<int> _parsedTriangles;
    int _recursionLim;
    int searchForVertexInUnfoldingSection( ObjModelPolyUnfolder*, int, int, int);

    ObjModelFaceUnfoldingVertexSearcher( const ObjModelFaceUnfoldingVertexSearcher&) = delete;
    void operator=( const ObjModelFaceUnfoldingVertexSearcher&) = delete;
};  // end class

}   // end namespace

#endif
