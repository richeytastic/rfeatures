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
    explicit ObjModelSurfacePointFinder( const ObjModel& m) : _model(m) {}

    // Finds the point on the surface of the model closest to input point t. vidx must be set as the
    // vertex from which to start searching over the surface for the point closest to t.
    // On return, point fv will either be in the plane of one of the polygons attached to vidx,
    // in which case fid will be set to the ID of this face and vidx will be -1, or fv will be
    // in the same position as vertex vidx in which case vidx will be unchanged and fid will be set
    // to some polygon ID attached to this vertex. Parameters fid and fv may be set to anything
    // (their correct values will be set upon return). Safe to pass in same argument as both t
    // and fv (if don't want to keep t). Returns squared l2-norm of (fv-t).
    double find( const cv::Vec3f& t, int& vidx, int& fid, cv::Vec3f& fv) const;

    /**
     * Same as above but the closet point to t on the surface is returned and the vertex ID and
     * polygon IDs are optional. If the starting vertex ID is not specified, an undefined model
     * vertex is used. This can result in the closest point on the surface being further away
     * than the absolute closest point because a local hill/valley may be preventing progress.
     */
    cv::Vec3f find( const cv::Vec3f& t, int vidx=-1, int* fid=nullptr) const;

private:
    const ObjModel& _model;
};  // end class

}   // end namespace

#endif
