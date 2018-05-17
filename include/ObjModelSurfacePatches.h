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

#ifndef RFEATURES_OBJ_MODEL_SURFACE_PATCHES_H
#define RFEATURES_OBJ_MODEL_SURFACE_PATCHES_H

#include "ObjModelKDTree.h"

namespace RFeatures {

class rFeatures_EXPORT ObjModelSurfacePatches
{
public:
    // Get points from patches around given locations within radius R.
    // Note that the model referenced by the kdtree must have connections between points!
    ObjModelSurfacePatches( const ObjModelKDTree*, float R);

    // Set pset with the M vertex IDs within R from v (if M < 0, get all points).
    // There may be fewer than M points within R of V, and so the actual number of vertex IDs
    // added to pset is returned. (pset is added to without first being cleared).
    // In the case that M > 0, a priority queue is used to ensure that the points
    // closest to v are found. Otherwise, a more efficient algorithm is used to
    // obtain all points within R of v.
    int getPatchVertexIds( const cv::Vec3f& v, IntSet& pset, int M=-1) const;

private:
    const ObjModelKDTree* _dtree;
    const float _sqR;
};  // end class

}   // end namespace

#endif
