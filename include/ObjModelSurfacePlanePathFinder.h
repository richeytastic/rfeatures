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

#ifndef RFEATURES_OBJ_MODEL_SURFACE_PLANE_PATH_FINDER_H
#define RFEATURES_OBJ_MODEL_SURFACE_PLANE_PATH_FINDER_H

/**
 * Finds the shortest string of points on a surface that lies within a plane defined by
 * the two path end points and a vector u that is WITHIN the plane and making a non-zero
 * inner product with the vector between the two points. Vector u specifies how 'tilted'
 * the path through the surface is. For a non-flat surface, choosing different directions
 * for u will change how curvy the surface path is and thus its length.
 * This suggests an optimisation algorithm for finding the shortest path total distance
 * by iteratively changing the orientation of the 'slicing' plane.
 *
 * If the default constructor is used an no slicing plane orientation is defined, the
 * plane orientation is changed based on the normal direction of the surface polygons
 * being sliced through. This results in surface 'curvature' finding path.
 *
 * In both constructors, if findShortest is true, path finding will continue after the first
 * path between the end points is found to see if there exists a shorter second path (i.e.,
 * there exists a complete loop around the model with the two end points being points on
 * this encircling loop). If there does exist a shorter second path, this path is set as
 * the path to be returned from lastPath() instead.
 */

#include "ObjModelSurfacePathFinder.h"

namespace RFeatures {

class rFeatures_EXPORT ObjModelSurfacePlanePathFinder : public ObjModelSurfacePathFinder
{
public:
    /**
     * Local surface curvature path finding (slicing plane changes wrt local polygon normals).
     */
    ObjModelSurfacePlanePathFinder( const ObjModel&, const ObjModelKDTree&, bool findShortest=false);

    /**
     * Fixed plane slicing path (path points will be incident with the fixed plane).
     * Vector u is incident with the intersecting plane. This is not the normal method
     * of defining a plane since the full parameterisation of the intersecting plane
     * can only be given when the two path endpoints are given. Vector u must make a
     * non-zero inner product with the path endpoints vector because the plane is
     * defined as going through p0 with normal n where n = u.cross(p0-p1) and p0,p1
     * are the path endpoint vectors. Note that for these purposes, the sign of u
     * is irrelevant.
     */
    ObjModelSurfacePlanePathFinder( const ObjModel&, const ObjModelKDTree&, const cv::Vec3f& u, bool findShortest=false);

    /**
     * Find a path on the model between the given endpoints which must be on the model's surface
     * returning the path sum and causing the path returned from lastPath() to be updated.
     * If a negative value is returned, no path could be found between the points.
     */
    double findPath( const cv::Vec3f& startPos, const cv::Vec3f& endPos) override;

private:
    const bool _findShortest;
    const bool _fixedPlane;
    const cv::Vec3f _u;

    cv::Vec3f _findInitialVertex( const cv::Vec3f&, int&) const;
};  // end class

}   // end namespace

#endif
