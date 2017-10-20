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

#include <ObjModel.h>
#include <list>


namespace RFeatures
{

typedef boost::unordered_set<Edge, HashEdge> EdgeSet;

class rFeatures_EXPORT VertexBoundaries
{
public:
    VertexBoundaries();
    ~VertexBoundaries();

    size_t getNumBoundaries() const;
    const std::list<int>& getBoundary( int i) const;

    void setEdge( int, int);    // Provide a pair of connected vertices (don't present same pair, in whatever order, more than once).
    void finish( const ObjModel::Ptr);  // Call after all boundaries set to ensure sub-boundary endpoints joined.

    size_t getNumEdgesParsed() const { return _edgeSet.size();}
    const EdgeSet& getEdgesParsed() const { return _edgeSet;}

    void sortBoundaries( bool maxFirst);
    void checkBoundaries( const ObjModel::Ptr) const;   // DEBUG

private:
    std::vector<const std::list<int>*> _blists;        // Completed boundaries
    boost::unordered_map<int, std::list<int>*> _front; // Boundaries indexed by "tail" vertex ([A] B C D)
    boost::unordered_map<int, std::list<int>*> _back;  // Boundaries indexed by "head" vertex (A B C [D])
    EdgeSet _edgeSet;     // Detecting duplicate edges

    bool checkAndSplice( int, int);
    void finishBoundary( int, int);
    VertexBoundaries( const VertexBoundaries&);
    void operator=( const VertexBoundaries&);
};  // end class

}   // end namespace

#endif

