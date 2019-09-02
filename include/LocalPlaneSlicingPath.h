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

#ifndef RFEATURES_LOCAL_PLANE_SLICING_PATH_H
#define RFEATURES_LOCAL_PLANE_SLICING_PATH_H

/**
 * Helper class for ObjModelSurfacePlanePathFinder.
 */

#include "PlaneSlicingPath.h"

namespace RFeatures {

class rFeatures_EXPORT LocalPlaneSlicingPath : public PlaneSlicingPath
{
public:
    // Point p must be in the plane of polygon fid. Point q is the endpoint of the path from p.
    LocalPlaneSlicingPath( const ObjModel& m, int fid, const cv::Vec3f& p)
        : PlaneSlicingPath( m, fid, p), _endPath(nullptr) {}

    void setEndPath( const LocalPlaneSlicingPath& spath) { _endPath = &spath;}

protected:
    cv::Vec3f polySlicingPlane( int, const cv::Vec3f&) const override;

private:
    const LocalPlaneSlicingPath *_endPath;
};  // end class

}   // end namespace

#endif
