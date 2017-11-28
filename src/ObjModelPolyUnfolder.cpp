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

#include <ObjModelPolyUnfolder.h>
using RFeatures::ObjModelPolyUnfolder;
using RFeatures::ObjModel;
using RFeatures::ObjPoly;
#include <cassert>


// public
ObjModelPolyUnfolder::ObjModelPolyUnfolder( const ObjModel::Ptr m, int T)
    : _model(m)
{
    const int* vidxs = _model->getFaceVertices(T);
    assert( vidxs);
    const cv::Vec3f& rv = _model->vtx(vidxs[0]);
    const cv::Vec3f& v0 = _model->vtx(vidxs[1]);
    const cv::Vec3f& v1 = _model->vtx(vidxs[2]);
    _unfoldedUVs[vidxs[0]] = rv;
    _unfoldedUVs[vidxs[1]] = v0;
    _unfoldedUVs[vidxs[2]] = v1;

    cv::Vec3d n0, n1;
    cv::normalize( v0 - rv, n0);
    cv::normalize( v1 - rv, n1);
    _planeNormal = n1.cross(n0);
}   // end ctor


// public
int ObjModelPolyUnfolder::unfoldAlongEdge( int T, int u0, int u1)
{
    assert( _unfoldedUVs.count(u0) > 0);
    assert( _unfoldedUVs.count(u1) > 0);

    const ObjPoly& face = _model->getFace( T);

    // u0 and u1 were already translated to be in the required plane (and are set in _unfoldedUVs) but
    // the position vector corresponding to the other vertex (u2) needs to be translated then "rotated" into the plane.
    const int u2 = face.getOpposite( u0, u1);
    const cv::Vec3d& v0 = _unfoldedUVs.at(u0);
    const cv::Vec3d& v1 = _unfoldedUVs.at(u1);

    // Get the translation vector needed to ensure that the position of u2 is relative to the new pseudo edge of T.
    const cv::Vec3d v2 = (cv::Vec3d)_model->vtx(u2) + v0 - (cv::Vec3d)_model->vtx(u0);

    cv::Vec3d e0 = v2 - v1;   // Edge opposite v0
    cv::Vec3d e1 = v2 - v0;   // Edge opposite v1
    cv::Vec3d e1u, e2u;
    cv::normalize( e1, e1u);
    cv::normalize( v1 - v0, e2u);   // Unit edge opposite v2 (may have to swap direction)

    cv::Vec3d tnrm = e2u.cross(e1u); // Calculate T's normal vector.
    // T's normal vector must point into the same half of space defined by the plane being rotated into.
    if ( tnrm.dot(_planeNormal) < 0)
    {
        cv::normalize( v0 - v1, e2u);
        tnrm = e2u.cross(e1u);
    }   // end if
    assert( tnrm.dot(_planeNormal) >= 0);

    // Calc unit vector orthogonal to the triangle normal and edge e2u.
    cv::Vec3d ou;
    cv::normalize( tnrm.cross(e2u), ou);    // ou is in plane of T and pointing out and away from the folding edge
    const double edgeLen = e1.dot(e2u);     // Projected length of v2-v0 along the folding edge
    const double outLen = e1.dot(ou);       // Projected length of v2-v0 along direction pointing out orthogonally from folding edge

    // Do the translation: new v2 in the required plane is outLen along Pvec + edgeLen along e + v0
    cv::Vec3d pvec; // NB since _planeNormal and e2u are at right angles (or should be), it is strictly not necessary to normalize.
    cv::normalize( _planeNormal.cross(e2u), pvec); // unit vector orthogonal to the required plane normal and edge e.
    _unfoldedUVs[u2] = v0 + outLen*pvec + edgeLen*e2u;
    return u2;
}   // end unfoldAlongEdge
