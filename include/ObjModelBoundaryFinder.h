#pragma once
#ifndef RFEATURES_OBJ_MODEL_BOUNDARY_FINDER_H
#define RFEATURES_OBJ_MODEL_BOUNDARY_FINDER_H

#include "ObjModel.h"
#include <list>
#include <vector>

namespace RFeatures
{

class rFeatures_EXPORT ObjModelBoundaryFinder
{
public:
    explicit ObjModelBoundaryFinder( const ObjModel::Ptr& objModel);

    // Find 2D boundaries on the model with vertices in connected order.
    // On return, boundary lists contain the ordered indices of the vertices for each
    // boundary on the model. Returns the number of separate boundaries found.
    // A return value of zero implies that the object has no 2D boundary.
    // IN ORDER TO WORK, THE MODEL MUST HAVE PURELY 2D VERTICES! (see ObjModelCleaner)
    int findOrderedBoundaryUniqueVertexIndices();

    int getNumBoundaries() const { return (int)_boundaries.size();}
    const std::list<int>& getBoundary( int b) const { return _boundaries.at(b);}

private:
    const ObjModel::Ptr& _model;
    std::vector< std::list<int> > _boundaries;
};  // end class

}   // end namespace

#endif
