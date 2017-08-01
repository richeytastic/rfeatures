#ifndef RFEATURES_DIJKSTRA_SHORTEST_PATH_FINDER_H
#define RFEATURES_DIJKSTRA_SHORTEST_PATH_FINDER_H

#include "ObjModel.h"
#include <vector>

namespace RFeatures
{

class rFeatures_EXPORT DijkstraShortestPathFinder
{
public:
    explicit DijkstraShortestPathFinder( const ObjModel::Ptr&);

    const ObjModel::Ptr& getObject() const { return _model;}

    // Sets the endpoints for the path to be found.
    // Returns false iff specified vertices are out of range.
    bool setEndPointVertexIndices( int endPoint, int startPoint);

    // Find the inclusive shortest path from startPoint to endPoint.
    // Returns the path length or -1 if no path could be found.
    // Number of entries pushed onto uvids will always be returned path length + 1.
    int findShortestPath( std::vector<int>& vids) const;

private:
    const ObjModel::Ptr _model;
    int _uA, _uB;
};  // end class

}   // end namespace

#endif
