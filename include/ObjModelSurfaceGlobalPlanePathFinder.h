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

#ifndef RFEATURES_OBJ_MODEL_SURFACE_GLOBAL_PLANE_PATH_FINDER_H
#define RFEATURES_OBJ_MODEL_SURFACE_GLOBAL_PLANE_PATH_FINDER_H

/**
 * Finds the shortest string of points on a surface that lies within a plane defined by
 * the two path end points and a fixed (global) vector which must be linearly independent
 * of the vector formed by the path end points. The slicing plane defines how 'tilted'
 * the path across the surface is. For a non-flat surface, choosing different directions
 * for u will change how curvy the surface path is and thus its length.
 * This suggests an optimisation algorithm for finding the shortest path (in the plane)
 * by iteratively changing the orientation of the fixed slicing plane.
 */

#include "ObjModelSurfacePathFinder.h"
#include "GlobalPlaneSlicingPath.h"

namespace RFeatures {

class rFeatures_EXPORT ObjModelSurfaceGlobalPlanePathFinder : public ObjModelSurfacePathFinder
{
public:
    /**
     * The slicing path uses a fixed (global) plane (path points will be incident with the plane).
     * Vector u defines the orientation of the view plane of the measurement (the slicing plane is
     * defined to be orthogonal to the view plane). Vector u should be linearly independent of the
     * path vector (p1-p0) since the slicing plane's orientation is defined as u.cross(p1-p0).
     */
    ObjModelSurfaceGlobalPlanePathFinder( const ObjModel&, const ObjModelKDTree&, const cv::Vec3f& u);

    /**
     * Find a path on the model between the given endpoints which must be on the model's surface
     * returning the path sum and causing the path returned from lastPath() to be updated.
     * If a negative value is returned, no path could be found between the points.
     */
    double findPath( const cv::Vec3f& startPos, const cv::Vec3f& endPos) override;

private:
    const cv::Vec3f _u;
};  // end class


/**
 * Take two paths stemming from a pair of end points and see if they join over the surface.
 * Returns > 0 if the given vector was filled with path vertices.
 */
rFeatures_EXPORT double findPathOption( PlaneSlicingPath&, int, PlaneSlicingPath&, int, std::vector<cv::Vec3f>&);


/**
 * Given two slicing paths, find and return the shortest (best) surface path that splices them together.
 */
rFeatures_EXPORT std::vector<cv::Vec3f> findBestPath( PlaneSlicingPath&, PlaneSlicingPath&);

/**
 * Convenience function that uses the ObjModelSurfacePointFinder to project a vertex into a polygon.
 * Returns the projected vertex and sets the id of the polygon it was projected into as out parameter fid.
 */
rFeatures_EXPORT cv::Vec3f findInitialVertex( const ObjModel&, const ObjModelKDTree&, const cv::Vec3f&, int &fid);

}   // end namespace

#endif
