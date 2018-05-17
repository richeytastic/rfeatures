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

    ObjModel::Ptr model() const { return _model;}

    // Returns whether this model is a triangulated manifold.
    bool isClean() const;

    // Since the model can be modified externally, use the clean function
    // to restore the model's cleaned state after actions that may result
    // in producing geometry that isn't a triangulated manifold. Returns
    // isClean(). If the model cannot be cleaned (returning false) the
    // component and boundary information will be reset and further calls
    // to components() or boundaries() will result in error().
    bool clean();

    const ObjModelComponentFinder& components() const;
    const ObjModelBoundaryFinder& boundaries() const;

private:
    ObjModel::Ptr _model;
    ObjModelIntegrityChecker _ic;
    ObjModelComponentFinder::Ptr _cf;
    ObjModelBoundaryFinder::Ptr _bf;
    void rebuildInfo();
    bool checkIntegrity();
    explicit ObjModelInfo( ObjModel::Ptr);
    ObjModelInfo( const ObjModelInfo&);     // No copy
    void operator=( const ObjModelInfo&);   // No copy
};  // end class

}   // end namespace

#endif
