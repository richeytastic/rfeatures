/************************************************************************
 * Copyright (C) 2019 Richard Palmer
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

#ifndef RFEATURES_OBJ_MODEL_HOLE_FILLER_H
#define RFEATURES_OBJ_MODEL_HOLE_FILLER_H

#include "ObjModel.h"

namespace RFeatures {

class rFeatures_EXPORT ObjModelHoleFiller
{
public:
    explicit ObjModelHoleFiller( ObjModel::Ptr);

    ObjModel::Ptr model() const { return _model;}

    // Fill the "hole" defined by the given list of ordered vertices. Every subsequent vertex must
    // be connected to its previous, and the front and back vertices in the list must be connected.
    // mpolys is the set of polygons for the manifold the hole is on. Note that on return, the set
    // of polygons for the manifold will be larger than this set so it will need to be regenerated.
    // Returns the number of polygons added to fill the hole.
    int fillHole( const std::list<int>&, const IntSet& mpolys);

    // Returns the set of newly added triangles resulting from the last call to fillHole.
    const IntSet& newPolys() const { return _npolys;}

private:
    ObjModel::Ptr _model;
    IntSet _npolys;
};  // end class

}   // end namespace

#endif
