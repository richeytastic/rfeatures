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
