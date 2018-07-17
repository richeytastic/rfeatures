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

#ifndef RFEATURES_OBJ_MODEL_INFO_H
#define RFEATURES_OBJ_MODEL_INFO_H

#include "ObjModelIntegrityChecker.h"
#include "ObjModelComponentFinder.h"
#include "ObjModelBoundaryFinder.h"

namespace RFeatures {

class rFeatures_EXPORT ObjModelInfo
{
public:
    typedef std::shared_ptr<ObjModelInfo> Ptr;
    // Checks integrity and cleans model only if necessary.
    // If a cleaned model cannot be created, returns null.
    // Multiple materials (if present) are merged into a single one.
    static Ptr create( ObjModel::Ptr);

    // Update with the given model, or if null, reset with the existing.
    // Returns false iff could not reset with a clean version of the model
    // in which case no changes are made. Cleaning is only undertaken if
    // necessary on models that aren't already triangulated manifolds.
    bool reset( ObjModel::Ptr m=nullptr);

    // Only when this function returns false is the model cleaned in reset.
    bool is2DManifold() const { return _ic.is2DManifold();}

    // Rediscover and overwrite boundary and component info for the model.
    // Does not check for integrity!
    void rebuildInfo();

    ObjModel::Ptr model() const { return _model;}
    const ObjModel* cmodel() const { return _model.get();}
    const ObjModelComponentFinder& components() const;
    const ObjModelBoundaryFinder& boundaries() const;

private:
    ObjModel::Ptr _model;
    ObjModelComponentFinder::Ptr _cf;
    ObjModelBoundaryFinder::Ptr _bf;
    ObjModelIntegrityChecker _ic;
    void clean( ObjModel::Ptr);
    bool checkIntegrity();
    explicit ObjModelInfo( ObjModel::Ptr);
    ObjModelInfo( const ObjModelInfo&) = delete;
    void operator=( const ObjModelInfo&) = delete;
};  // end class

}   // end namespace

#endif
