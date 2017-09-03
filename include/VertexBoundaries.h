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
 * Needed for boundary mappers to record the list of vertices from a
 * ObjModel instance that comprise a boundary. Used in:
 * ObjModelBoundaryFinder
 * ObjModelCropper
 */

#ifndef RFEATURES_VERTEX_BOUNDARIES_H
#define RFEATURES_VERTEX_BOUNDARIES_H

#include "rFeatures_Export.h"
#include <boost/unordered_map.hpp>
#include <vector>
#include <list>


namespace RFeatures
{

class rFeatures_EXPORT VertexBoundaries
{
public:
    ~VertexBoundaries();

    size_t getNumBoundaries() const;
    const std::list<int>& getBoundary( int i) const;

    void setEdge( int r, int a);

    void sortBoundaries( bool maxFirst);

private:
    std::vector<std::list<int>*> _blists;                  // Holds completed boundaries
    boost::unordered_map<int, std::list<int>*> _subfirst;  // Boundaries indexed by "first" vertex
    boost::unordered_map<int, std::list<int>*> _sublast;   // Boundaries indexed by "last" vertex

    int checkForCompletedBoundary( int i);
    int checkForSubBoundarySplicing( int i);
};  // end class

}   // end namespace

#endif

