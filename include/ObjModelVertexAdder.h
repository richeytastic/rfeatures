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

namespace RFeatures {

class rFeatures_EXPORT ObjModelVertexAdder
{
public:
    explicit ObjModelVertexAdder( ObjModel::Ptr);

    // Adds vertices until there are no more triangles in the object
    // having area greater than maxTriangleArea. Returns the number
    // of new vertices added.
    int addVerticesToMaxTriangleArea( double maxTriangleArea);

    // Subdivides triangles having areas larger than given, but then merges
    // newly created triangles along existing edges - joinging together pairs
    // of new introduced vertices. This DOES change the morphology of the
    // object (making it flatter on average), but tends to distribute
    // vertices into more equally spaced locations.
    int subdivideAndMerge( double maxTriangleArea);

private:
    ObjModel::Ptr _model;
};  // end class

}   // end namespace

#endif
