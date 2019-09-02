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

#include <LocalPlaneSlicingPath.h>
using RFeatures::LocalPlaneSlicingPath;


cv::Vec3f LocalPlaneSlicingPath::polySlicingPlane( int fid, const cv::Vec3f& v) const
{
    assert(_endPath);
    cv::Vec3f u;
    const cv::Vec3f fn = model().calcFaceNorm(fid);
    const cv::Vec3f& q = _endPath->lastVertexAdded();
    cv::normalize( (q-v).cross(fn), u);
    return u;
}   // end polySlicingPlane
