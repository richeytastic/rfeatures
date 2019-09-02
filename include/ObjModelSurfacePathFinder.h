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

#ifndef RFEATURES_OBJ_MODEL_SURFACE_PATH_FINDER_H
#define RFEATURES_OBJ_MODEL_SURFACE_PATH_FINDER_H

/**
 * Finds a path over the surface of the given model between a start and end point.
 * The default implementation of findPath uses DijkstraShortestPathFinder.
 */

#include "ObjModelKDTree.h"

namespace RFeatures {

class rFeatures_EXPORT ObjModelSurfacePathFinder
{
public:
    ObjModelSurfacePathFinder( const ObjModel&, const ObjModelKDTree&);

    // Return the last path calculated from findPath. All points lie on the surface
    // of the model. The first and last points will be the points on the model closest
    // to the start and end points provided to findPath. If an empty vector is returned,
    // no calls to findPath have been received.
    const std::vector<cv::Vec3f>& lastPath() const { return _lpath;}

    /**
     * Find a path between the given endpoints returning the path sum and updating the path returned from lastPath().
     * If a negative value is returned, no path could be found between the points.
     */
    virtual double findPath( const cv::Vec3f& startPos, const cv::Vec3f& endPos);

    static double calcPathLength( const std::vector<cv::Vec3f>&);

protected:
    const ObjModel& _model;
    const ObjModelKDTree& _kdt;
    std::vector<cv::Vec3f> _lpath;
};  // end class

}   // end namespace

#endif
