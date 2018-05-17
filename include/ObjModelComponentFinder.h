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

#ifndef RFEATURES_OBJ_MODEL_COMPONENT_FINDER_H
#define RFEATURES_OBJ_MODEL_COMPONENT_FINDER_H

#include "ObjModelBoundaryFinder.h"

/**
 * Find separate components of a model from its already discovered boundaries.
 */
namespace RFeatures {

class rFeatures_EXPORT ObjModelComponentFinder
{
public:
    typedef boost::shared_ptr<ObjModelComponentFinder> Ptr;
    static Ptr create( const ObjModelBoundaryFinder::Ptr);

    // Return the boundary finder and object passed in as parameter.
    const ObjModel::Ptr getObject() const { return _bf->getObject();}
    const ObjModelBoundaryFinder::Ptr boundaryFinder() const { return _bf;}

    size_t findComponents();    // Returns # of components discovered.

    size_t size() const { return _components.size();} // Returns # components.
    size_t numComponents() const { return size();}    // Synonymous with size().

    // Returns number of boundaries on the given component. This can be zero for
    // components with no edges (e.g. a ball). Returns -1 for out of range index.
    int numComponentBoundaries( int) const;

    // Returns component (as a set of face IDs) at position i. Components are stored in
    // descending order of # polygons. In the case of the model having only a single
    // component, this function returns &getObject()->getFaceIds(). By default, returns the
    // largest (and possibly only) model component. Returns NULL if index out of range.
    const IntSet* component( int i=0) const;

    // Returns the set containing the boundary indices for the given component.
    // The lowest value index is the boundary with the longest list of vertices.
    // Returns NULL if index out of range or if the component has no boundaries.
    const IntSet* cboundaries( int i) const;

    // Returns longest boundary index for component c or -1 for components with no boundary.
    // Returns the longest boundary index on the largest component by default.
    // Returns -2 if given component index out of range.
    int lboundary( int c=0) const;

private:
    const ObjModelBoundaryFinder::Ptr _bf;
    std::vector<const IntSet*> _components;         // Component sets are face IDs
    std::unordered_map<const IntSet*, IntSet> _cb;  // Boundary indices keyed by component
    std::unordered_map<const IntSet*, int> _lb;     // Longest boundary index for each component

    explicit ObjModelComponentFinder( const ObjModelBoundaryFinder::Ptr);
    virtual ~ObjModelComponentFinder();
    ObjModelComponentFinder( const ObjModelComponentFinder&); // No copy
    void operator=( const ObjModelComponentFinder&);         // No copy
};  // end class

}   // end namespace

#endif
