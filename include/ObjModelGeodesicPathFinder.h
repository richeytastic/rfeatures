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

#ifndef RFEATURES_OBJ_MODEL_GEODESIC_PATH_FINDER_H
#define RFEATURES_OBJ_MODEL_GEODESIC_PATH_FINDER_H

#include "ObjModelKDTree.h"

namespace RFeatures
{

class rFeatures_EXPORT ObjModelGeodesicPathFinder
{
public:
    ObjModelGeodesicPathFinder( const ObjModel::Ptr);
    ObjModelGeodesicPathFinder( const ObjModelKDTree::Ptr);    

    // Find geodesic over surface starting at arbitrary points. Only available if built using 2nd ctor.
    // Find the geodesic path on the model's surface from v0 to v1, placing the output points in pts.
    // Returns the number of points added to pts. Does not clear pts before use!
    int findGeodesic( const cv::Vec3f& v0, const cv::Vec3f& v1, std::vector<cv::Vec3f>& pts);

    // As above but endpoints defined to be at the specified vertices (so KD tree not needed).
    int findGeodesic( int v0, int v1, std::vector<cv::Vec3f>& pts);

private:
    const ObjModel::Ptr _model;
    const ObjModelKDTree::Ptr _kdtree;
};  // end class

}   // end namespace

#endif
