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
 * Replace tetrahedrons in the model. Only works with triangulated meshes!
 */

#ifndef RFEATURES_OBJ_MODEL_TETRAHEDRON_REPLACER_H
#define RFEATURES_OBJ_MODEL_TETRAHEDRON_REPLACER_H

#include "ObjModel.h"

namespace RFeatures
{

class rFeatures_EXPORT ObjModelTetrahedronReplacer
{
public:
    ObjModelTetrahedronReplacer( ObjModel::Ptr);

    // Finds vertices that are the points of three adjacent polygons and removes these polygons
    // replacing them with a single triangle (if one does not already exist). Returns the number
    // of vertices removed.
    int removeTetrahedrons();

private:
    ObjModel::Ptr _model;
};  // end class

}   // end namespace

#endif

