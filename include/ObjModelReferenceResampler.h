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

#ifndef RFEATURES_OBJ_MODEL_REFERENCE_RESAMPLER_H
#define RFEATURES_OBJ_MODEL_REFERENCE_RESAMPLER_H

/**
 * Uses minimal sets of coregistered points to sample surface points on a source mesh
 * to match a target mesh's vertices; the output mesh having the same coregistered
 * vertex IDs and connectivity as the target mesh.
 */
#include "ObjModelKDTree.h"

namespace RFeatures {

class rFeatures_EXPORT ObjModelReferenceResampler
{
public:
    // The mesh to sample against is given by tmod. Model tref defines a minimal set of landmark
    // points - only vertices are required (connectivity is ignored). Parameter sref in the sample
    // function must have points coregistered to have the same vertex indices.
    // Parameter k specifies the number of landmark triples to use in estimating the
    // position of a sampled point.
    ObjModelReferenceResampler( const ObjModel* tmod, const ObjModel* tref, int k=3);

    // Sample the vertices of the parameter model to return a mesh with the same
    // vertex indices as the target (constructor) mesh and the same connectivity.
    ObjModel::Ptr sample( const ObjModelKDTree*, const ObjModel* sref) const;

private:
    const ObjModel* _tgt;
    int _k;
    ObjModelKDTree::Ptr _tlmks;
};  // end class

}   // end namespace

#endif
