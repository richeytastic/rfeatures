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
    // If a cleaned model cannot be created, returns NULL.
    static Ptr create( ObjModel::Ptr);

    // Update with the given model, or if NULL, reset with the existing.
    // Returns false iff could reset with a clean version of the model.
    // Cleaning is only undertaken if necessary on models that don't
    // already consist of a triangulated manifold.
    bool reset( ObjModel::Ptr m=NULL);

    ObjModel::Ptr model() const { return _model;}
    const ObjModel* cmodel() const { return _model.get();}
    const ObjModelComponentFinder& components() const;
    const ObjModelBoundaryFinder& boundaries() const;

private:
    ObjModel::Ptr _model;
    ObjModelComponentFinder::Ptr _cf;
    ObjModelBoundaryFinder::Ptr _bf;
    ObjModelIntegrityChecker _ic;
    void clean();
    void rebuildInfo();
    bool checkIntegrity();
    explicit ObjModelInfo( ObjModel::Ptr);
    ObjModelInfo( const ObjModelInfo&);     // No copy
    void operator=( const ObjModelInfo&);   // No copy
};  // end class

}   // end namespace

#endif
