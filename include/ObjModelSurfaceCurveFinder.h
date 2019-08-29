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

#ifndef RFEATURES_OBJ_MODEL_SURFACE_CURVE_FINDER_H
#define RFEATURES_OBJ_MODEL_SURFACE_CURVE_FINDER_H

#include "ObjModelSurfacePathFinder.h"

namespace RFeatures {

class rFeatures_EXPORT ObjModelSurfaceCurveFinder : public ObjModelSurfacePathFinder
{
public:
    ObjModelSurfaceCurveFinder( const ObjModel&, const ObjModelKDTree&);

    // Creates a path between v0 and v1 where the orientation of the curve is determined
    // by the local curvature of every polygon crossed between the two points.
    // Returns the length of the path if found or a negative value if not found.
    double findPath( const cv::Vec3f& v0, const cv::Vec3f& v1) override;

private:
    int _getOppositeEdge( const cv::Vec3d&, const cv::Vec3d&, int) const;
};  // end class

}   // end namespace

#endif
