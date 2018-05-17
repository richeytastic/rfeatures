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

#ifndef RFEATURES_DISTANCE_MEASURER_H
#define RFEATURES_DISTANCE_MEASURER_H

#include "DijkstraShortestPathFinder.h"

namespace RFeatures {

class rFeatures_EXPORT DistanceMeasurer
{
public:
    explicit DistanceMeasurer( const ObjModel* om);

    // Given the shortest path between points v0 and v1 over the model, return
    // the point x on that path that maximises d(v0 - x) + d(v1 - x) where d is the L2-norm.
    cv::Vec3f getMaximallyExtrudedPoint( const cv::Vec3f& v0, const cv::Vec3f& v1) const;

    // Same as above but specifying vertex IDs.
    cv::Vec3f getMaximallyExtrudedPoint( int vid0, int vid1) const;

    // Given a vector of vertex IDs, return the index of the element in vids for
    // the point x that maximises d(v0 - x) + d(v1 - x) where d is the L2-norm.
    int getMaximallyExtrudedPointIndex( const std::vector<int>& vids) const;

private:
    const ObjModel* _om;
};  // end class

}   // end namespace

#endif
