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

#ifndef RFEATURES_OBJ_MODEL_SURFACE_LOCAL_PLANE_PATH_FINDER_H
#define RFEATURES_OBJ_MODEL_SURFACE_LOCAL_PLANE_PATH_FINDER_H

#include "ObjModelSurfacePathFinder.h"
#include "LocalPlaneSlicingPath.h"

namespace RFeatures {

class rFeatures_EXPORT ObjModelSurfaceLocalPlanePathFinder : public ObjModelSurfacePathFinder
{
public:
    /**
     * Local (per polygon) slicing planes derived from the normal direction of each polygon are used to
     * define the path direction across the surface of the model. This causes the path to meander based
     * on the changing orientation of adjacent polygons being crossed. The path will only be straight
     * if all crossed polygons have are flat with respect to one another.
     */
    ObjModelSurfaceLocalPlanePathFinder( const ObjModel&, const ObjModelKDTree&);

    /**
     * Find a path on the model between the given endpoints which must be on the model's surface
     * returning the path sum and causing the path returned from lastPath() to be updated.
     * If a negative value is returned, no path could be found between the points.
     */
    double findPath( const cv::Vec3f& startPos, const cv::Vec3f& endPos) override;
};  // end class

}   // end namespace

#endif
