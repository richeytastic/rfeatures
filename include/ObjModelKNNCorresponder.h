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

#ifndef RFEATURES_OBJ_MODEL_KNN_CORRESPONDER_H
#define RFEATURES_OBJ_MODEL_KNN_CORRESPONDER_H

/**
 * Does distance weighted K nearest neighbour on a target mesh against a known mesh to
 * produce a new vertex coregistered point cloud (no connectivity).
 */
#include "ObjModelKDTree.h"

namespace RFeatures {

class rFeatures_EXPORT ObjModelKNNCorresponder
{
public:
    // Mask must have its vertex IDs on [0,n) where n = mask->numVertices().
    ObjModelKNNCorresponder( const ObjModel& mask);

    // Return the N x M affinity matrix where N is the number of points in the mask,
    // and M the number of points in the given target mesh. Each entry is the inverse of the
    // squared distance between the mask point and the target point. Only vertices from the
    // target mesh that are found to be within K nearest neighbours of the mask vertices
    // have non-zero entries. The coregistered points C can be calculated from provided target
    // vertices Y and the returned matrix A as C = AY. That is, each point coregistered to
    // the mask is the weighted sum of vertices in Y.
    cv::SparseMat_<float> sample( const ObjModelKDTree& Y, int k) const;

private:
    const ObjModel& _mask;
};  // end class

}   // end namespace

#endif
