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

#ifndef RFEATURES_GLOBAL_PLANE_SLICING_PATH_H
#define RFEATURES_GLOBAL_PLANE_SLICING_PATH_H

/**
 * Helper class for ObjModelSurfacePlanePathFinder.
 */

#include "PlaneSlicingPath.h"

namespace RFeatures {

class rFeatures_EXPORT GlobalPlaneSlicingPath : public PlaneSlicingPath
{
public:
    // Vector u defines the slicing plane's orientation.
    GlobalPlaneSlicingPath( const ObjModel& m, int fid, const cv::Vec3f& v, const cv::Vec3f& u)
        : PlaneSlicingPath( m, fid, v), _u(u)
    {
        assert( cv::norm(u) > 0);
    }   // end ctor

protected:
    inline cv::Vec3f polySlicingPlane( int, const cv::Vec3f&) const override { return _u;}

private:
    const cv::Vec3f _u;
};  // end class

}   // end namespace

#endif
