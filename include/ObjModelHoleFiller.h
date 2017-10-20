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

#ifndef RFEATURES_OBJ_MODEL_HOLE_FILLER_H
#define RFEATURES_OBJ_MODEL_HOLE_FILLER_H

#include "ObjModel.h"   // RFeatures

namespace RFeatures
{

class rFeatures_EXPORT ObjModelHoleFiller
{
public:
    explicit ObjModelHoleFiller( ObjModel::Ptr);

    // Fills holes in the model starting at the component connected
    // to vertex svid (model must be a triangulated mesh!)
    int fillHoles( int svid=0);

private:
    ObjModel::Ptr _model;
};  // end class

}   // end namespace

#endif
