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

#ifndef RFEATURES_OBJ_POLY_PLANE_H
#define RFEATURES_OBJ_POLY_PLANE_H

/**
 * Find how to split an ObjPoly into two pieces and know which side is on the "inside" of
 * a space divided by an arbitrary plane.
 */

#include "ObjModel.h"

namespace RFeatures {

class rFeatures_EXPORT ObjPolyPlane
{
public:
    // f is the face to test, p is a point on the plane and n is a normal vector
    // giving the plane's orientation with it pointing in the direction of "inside"
    // the half space we want to keep.
    ObjPolyPlane( const ObjModel* src, int fid, const cv::Vec3d& p, const cv::Vec3d& n);

    // Returns -1 if all vertices on this face are in the "outside" half, 1 if all vertices are in the "inside" half
    // and 0 if the vertices stradle the plane (then use findPlaneVertices to find where the plane intersects the
    // edges of the triangle). If 0 is returned, then a() is the vertex that is by itself on one side of the plane
    // (with b() and c() the other two vertices). With zero returned, if inside() returns true, then a() is
    // the vertex that is in the "inside" half of the space that the plane divides.
    inline int inhalf() const { return _nih;}

    // Only valid to call if inhalf has returned zero, this says if vertex a is within
    // the half space that the initially provided vector n pointed into.
    inline bool inside() const { return _inside;}

    inline int a() const { return _a;}
    inline int b() const { return _b;}
    inline int c() const { return _c;}

    inline const cv::Vec3f& va() const { return _mod->vtx(_fvidxs[_a]);}
    inline const cv::Vec3f& vb() const { return _mod->vtx(_fvidxs[_b]);}
    inline const cv::Vec3f& vc() const { return _mod->vtx(_fvidxs[_c]);}

    inline const cv::Vec2f& uva() const { return _mod->faceUV(_fid, _a);}
    inline const cv::Vec2f& uvb() const { return _mod->faceUV(_fid, _b);}
    inline const cv::Vec2f& uvc() const { return _mod->faceUV(_fid, _c);}

    // Set the vertices that intersect with the plane as yb and yc.
    // It is only valid to call this function if inhalf() has returned zero.
    void findPlaneVertices( cv::Vec3f& yb, cv::Vec3f& yc) const;

private:
    const ObjModel* _mod;
    int _fid;
    const int* _fvidxs;
    const cv::Vec3d _p;
    cv::Vec3d _n;
    bool _inside;
    int _a, _b, _c;
    int _nih;

    // Returns -1 if all vertices of the face are in the wrong half of the space.
    // Returns 1 if all vertices of the face are in the right half of the space.
    // Returns 0 if face crosses the boundary, sets a to be the index into fvidxs on the
    // side in which only that single vertex resides, and sets the sign of n to point
    // into the half that this vertex resides.
    int _vertexInHalf();    // On return, a is on {0,1,2}
};  // end class

}   // end namespace

#endif
