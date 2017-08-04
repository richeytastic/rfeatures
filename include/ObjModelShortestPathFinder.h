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

#ifndef RFEATURES_OBJ_MODEL_SHORTEST_PATH_FINDER_H
#define RFEATURES_OBJ_MODEL_SHORTEST_PATH_FINDER_H

#include "ObjModelKDTree.h"

namespace RFeatures
{

class rFeatures_EXPORT ObjModelShortestPathFinder
{
public:
    explicit ObjModelShortestPathFinder( const ObjModelKDTree::Ptr);

    const ObjModel::Ptr& getKDTree() const { return _kdtree;}

    // Find the shortest path on the model's surface from v0 to v1, placing the output points in pts.
    // Returns the number of points added to pts. Does not clear pts before use!
    int operator()( const cv::Vec3f& v0, const cv::Vec3f& v1, std::vector<cv::Vec3f>& pts) const;

    // As above but endpoints defined to be at the specified vertices.
    int operator()( int v0, int v1, std::vector<cv::Vec3f>& pts) const;

private:
    const ObjModelKDTree::Ptr _kdtree;
};  // end class

}   // end namespace

#endif
