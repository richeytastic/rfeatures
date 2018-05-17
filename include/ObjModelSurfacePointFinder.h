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
 * A linear search for points on the surface of a model starting from some arbitrary vertex.
 * Richard Palmer
 * July 2017
 */

#ifndef RFEATURES_OBJ_MODEL_SURFACE_POINT_FINDER_H
#define RFEATURES_OBJ_MODEL_SURFACE_POINT_FINDER_H

#include "ObjModel.h"

namespace RFeatures {

class rFeatures_EXPORT ObjModelSurfacePointFinder
{
public:
    ObjModelSurfacePointFinder( const ObjModel*);

    const ObjModel* model() const { return _model;}

    // Finds the point fv on the surface of the model closest to input vertex v.
    // On return, point fv will either be in the plane of one of the polygons attached to vidx,
    // in which case fid will be set to the index of this face and vidx will be -1, or fv will be
    // in the same position as vertex vidx in which case vidx will be unchanged (same as input vertex)
    // and fid will be set to -1. The starting input vertex vidx must be given. Parameters fid and
    // fv may be set to anything (their correct values will be set on return). Returns squared l2-norm of (fv-v).
    double find( const cv::Vec3f& v, int& vidx, int& fid, cv::Vec3f& fv) const;

private:
    const ObjModel* _model;
};  // end class

}   // end namespace

#endif
