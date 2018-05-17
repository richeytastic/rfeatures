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

#ifndef RFEATURES_OBJ_MODEL_BOUNDARY_FINDER_H
#define RFEATURES_OBJ_MODEL_BOUNDARY_FINDER_H

#include "ObjModel.h"

/**
 * Find 2D boundaries on the model with vertices in connected order.
 */
namespace RFeatures {

class rFeatures_EXPORT ObjModelBoundaryFinder
{
public:
    typedef std::shared_ptr<ObjModelBoundaryFinder> Ptr;
    static Ptr create( const ObjModel*);

    const ObjModel* model() const { return _model;}

    // Functions return the number of boundaries discovered.
    size_t findOrderedBoundaryVertices( const IntSet& boundaryVertices);    // Copy in given set of boundary vertices
    size_t findOrderedBoundaryVertices();   // Discover set of boundary vertices first

    // Boundaries are stored in descending order of number of vertices.
    const std::list<int>& boundary( int i) const { return _boundaries[i];}
    size_t size() const { return _boundaries.size();}   // Returns # boundaries.
    size_t numBoundaries() const { return size();}      // Synonymous with size().

private:
    const ObjModel* _model;
    std::vector< std::list<int> > _boundaries;

    explicit ObjModelBoundaryFinder( const ObjModel*);
    virtual ~ObjModelBoundaryFinder() {}
    ObjModelBoundaryFinder( const ObjModelBoundaryFinder&); // No copy
    void operator=( const ObjModelBoundaryFinder&);         // No copy
};  // end class

}   // end namespace

#endif
