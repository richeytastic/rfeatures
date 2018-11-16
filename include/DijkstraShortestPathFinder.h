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

/**
 * A* modified generic version of Dijkstra's algorithm for finding shortest paths in Euclidean space.
 * PathCostCalculator may defines path costs between vertices (default is l2-norm^2).
 **/

#ifndef RFEATURES_DIJKSTRA_SHORTEST_PATH_FINDER_H
#define RFEATURES_DIJKSTRA_SHORTEST_PATH_FINDER_H

#include "ObjModel.h"

namespace RFeatures {

// Default cost between two vertices is l2-norm^2.
class rFeatures_EXPORT PathCostCalculator
{
public:
    virtual ~PathCostCalculator(){}
    // Returns non-negative cost between two points.
    virtual double operator()( const cv::Vec3d&, const cv::Vec3d&) const;
};  // end class


class rFeatures_EXPORT DijkstraShortestPathFinder
{
public:
    // If not path cost calculator explicitly given, the default (l2-norm) is used.
    DijkstraShortestPathFinder( const ObjModel*, PathCostCalculator* pcf=nullptr);
    virtual ~DijkstraShortestPathFinder();

    const ObjModel* model() const { return _model;}

    // Sets the endpoints for the path to be found.
    // Returns false iff specified vertices are out of range.
    bool setEndPointVertexIndices( int endPoint, int startPoint);

    // Find the inclusive shortest path from startPoint to endPoint.
    // Returns the path length which is equivalent to S-1 where S is the
    // number of vertex indices appended to vids (if S=0, returned path length
    // will be zero, not -1). If clearVector is set true, the provided vector will
    // first be cleared before vertex indices are appended. If set false, vertex
    // indices appended to vids will NOT include duplicate endpoints (i.e. multiple
    // calls to this function using the same parameter vids with interleaved calls
    // to setEndPointVertexIndices of, for example, (b,a), (c,b), (d,c), will NOT
    // result in multiple adjacent insertions into vids of vertex indices b and c).
    int findShortestPath( std::vector<int>& vids, bool clearVector=true) const;

private:
    const ObjModel *_model;
    PathCostCalculator *_pcc;
    bool _delpcc;
    int _uA, _uB;

    DijkstraShortestPathFinder( const DijkstraShortestPathFinder&) = delete;
    void operator=( const DijkstraShortestPathFinder&) = delete;
};  // end class

}   // end namespace

#endif
