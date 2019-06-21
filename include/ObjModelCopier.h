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

#ifndef RFEATURES_OBJ_MODEL_COPIER_H
#define RFEATURES_OBJ_MODEL_COPIER_H

/**
 * Copy a subsection of a model by specifying the individual triangles to copy.
 * The source object's transform matrix is also copied over.
 * For making full copies, use ObjModel::deepCopy instead.
 **/

#include "ObjModel.h"

namespace RFeatures {

class rFeatures_EXPORT ObjModelCopier
{
public:
    /**
     * Create a new empty model that triangles can be progressively copied to.
     * Copies over the source model's transformation matrix.
     */
    explicit ObjModelCopier( const ObjModel& source);

    /**
     * Copy over the given triangle to the output model.
     */
    void add( int fid);

    /**
     * Returns the new model.
     */
    ObjModel::Ptr copiedModel() const { return _cmodel;}

private:
    const ObjModel& _model;
    ObjModel::Ptr _cmodel;
    ObjModelCopier( const ObjModelCopier&) = delete;
    void operator=( const ObjModelCopier&) = delete;
};  // end class

}   // end namespace

#endif
