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

#ifndef RFEATURES_OBJ_MODEL_BOUNDARY_FINDER2_H
#define RFEATURES_OBJ_MODEL_BOUNDARY_FINDER2_H

#include "ObjModel.h"

/**
 * Find 2D boundaries on the model with vertices in connected order.
 */
namespace RFeatures
{

class rFeatures_EXPORT ObjModelBoundaryFinder2
{
public:
    explicit ObjModelBoundaryFinder2( const ObjModel::Ptr);

    int findOrderedBoundaryVertices();

    void sortBoundaries( bool maxFirst=true);

    const std::list<int>& getBoundary(int i) const { return _boundaries[i];}

private:
    const ObjModel::Ptr _model;
    std::vector< std::list<int> > _boundaries;

    ObjModelBoundaryFinder2( const ObjModelBoundaryFinder2&); // No copy
    void operator=( const ObjModelBoundaryFinder2&);     // No copy
};  // end class

}   // end namespace

#endif
