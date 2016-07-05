#pragma once
#ifndef RFEATURES_DIJKSTRA_SHORTEST_PATH_FINDER_H
#define RFEATURES_DIJKSTRA_SHORTEST_PATH_FINDER_H

#include "ObjModel.h"
#include <vector>

namespace RFeatures
{

class rFeatures_EXPORT DijkstraShortestPathFinder
{
public:
    explicit DijkstraShortestPathFinder( const ObjModel::Ptr& om);

    // Sets the endpoints for the path to be found.
    // Returns false iff specified unique vertices are out of range.
    bool setEndPointUniqueVertexIndices( int endPoint, int startPoint);

    // Find the inclusive shortest path from startPoint to endPoint.
    // Returns the path length or -1 if no path could be found.
    // Number of entries pushed onto uvids will always be returned path length + 1.
    int findShortestPath( std::vector<int>& uvids) const;

private:
    const ObjModel::Ptr _om;
    int _uA, _uB;
};  // end class

}   // end namespace

#endif
