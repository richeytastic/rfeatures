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

#ifndef RFEATURES_OBJ_MODEL_VERTEX_ADDER_H
#define RFEATURES_OBJ_MODEL_VERTEX_ADDER_H

#include "ObjModel.h"

namespace RFeatures
{

class rFeatures_EXPORT ObjModelVertexAdder
{
public:
    // Adds a bunch more extra vertices to edges that are longer than minDistance.
    // On return, the provided model will not have any triangle edges longer than
    // minDistance. Apart from subdividing the model's triangles (and adding more
    // complexity), this does nothing to change the morphology of the model.
    // Returns the number of new vertices added.
    static int addVertices( ObjModel::Ptr, double minDistance);

};  // end class

}   // end namespace

#endif
