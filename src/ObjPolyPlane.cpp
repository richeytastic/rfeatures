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

#include <ObjPolyPlane.h>
#include <FeatureUtils.h>
#include <cmath>
using RFeatures::ObjPolyPlane;
using RFeatures::ObjModel;


ObjPolyPlane::ObjPolyPlane( const ObjModel& src, int fid, const cv::Vec3d& p, const cv::Vec3d& n)
    : _mod(src), _fid(fid), _fvidxs( _mod.fvidxs(fid)), _p(p), _n(n), _inside(true), _nih(_vertexInHalf()) { }   // end ctor


void ObjPolyPlane::findPlaneVertices( cv::Vec3f& yb, cv::Vec3f& yc) const
{
    const cv::Vec3f& xa = _mod.vtx(_fvidxs[_a]);
    const cv::Vec3f& xb = _mod.vtx(_fvidxs[_b]);
    const cv::Vec3f& xc = _mod.vtx(_fvidxs[_c]);
    yb = RFeatures::linePlaneIntersection( _p, _n, xa, xb);   // Incident with plane and xa,db
    yc = RFeatures::linePlaneIntersection( _p, _n, xa, xc);   // Incident with plane and xa,xc
}   // end findPlaneVertices


// Returns -1 if all vertices of the face are in the wrong half of the space.
// Returns 1 if all vertices of the face are in the right half of the space.
// Returns 0 if face crosses the boundary, sets a to be the index into fvidxs on the
// side in which only that single vertex resides, and sets the sign of n to point
// into the half that this vertex resides.
int ObjPolyPlane::_vertexInHalf()   // On return, a is on {0,1,2}
{
    const cv::Vec3f& x0 = _mod.vtx(_fvidxs[0]);
    const cv::Vec3f& x1 = _mod.vtx(_fvidxs[1]);
    const cv::Vec3f& x2 = _mod.vtx(_fvidxs[2]);

    const cv::Vec3f fp = _p;
    const double dx0 = _n.dot(x0 - fp);
    const double dx1 = _n.dot(x1 - fp);
    const double dx2 = _n.dot(x2 - fp);

    const int s0 = int(copysign( 1, dx0));
    const int s1 = int(copysign( 1, dx1));
    const int s2 = int(copysign( 1, dx2));

    if ( fabs(s0 + s1 + s2) == 3)
        return s0;  // Will be (+/-)1.

    if ( fabs(s0 + s1) == 2)
    {
        _a = 2;
        _n *= s2;
        _inside = s2 > 0;
    }   // end if
    else if ( fabs(s1 + s2) == 2)
    {
        _a = 0;
        _n *= s0;
        _inside = s0 > 0;
    }   // end else if
    else
    {
        _a = 1;
        _n *= s1;
        _inside = s1 > 0;
    }   // end else

    _b = (_a+1)%3;
    _c = (_a+2)%3;
    return 0;
}   // end _vertexInHalf
