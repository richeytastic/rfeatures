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

#include <ObjModelTools.h>
using RFeatures::ObjModelPolyUnfolder;
using RFeatures::ObjModel;
using RFeatures::ObjPoly;
#include <cassert>


ObjModelPolyUnfolder::ObjModelPolyUnfolder( const ObjModel* model) : _model(model), _pnorm(0,0,0) {}


ObjModelPolyUnfolder::ObjModelPolyUnfolder( const ObjModel* model, int T) : _model(model), _pnorm(0,0,0)
{
    reset(T);
}   // end ctor


ObjModelPolyUnfolder::ObjModelPolyUnfolder( const ObjModelPolyUnfolder& unf, int T)
    : _model(unf.model()), _pnorm(unf.norm())
{
    const int* vidxs = _model->getFaceVertices(T);
    if ( vidxs)
    {
        const int r = vidxs[0];
        const int a = vidxs[1];
        const int b = vidxs[2];
        _uvtxs[r] = unf.uvtx(r);
        _uvtxs[a] = unf.uvtx(a);
        _uvtxs[b] = unf.uvtx(b);
        _upolys.insert(T);
    }   // end if
}   // end ctor


cv::Vec3d ObjModelPolyUnfolder::calcUnfoldedPoint( int T, const cv::Vec3f& p) const
{
    assert( isPolyUnfolded(T));
    const cv::Vec3f pp = _model->toPropFromAbs( T, p);
    const int* vidxs = _model->fvidxs(T);
    const cv::Vec3d& v0 = uvtx( vidxs[0]);
    const cv::Vec3d& v1 = uvtx( vidxs[1]);
    const cv::Vec3d& v2 = uvtx( vidxs[2]);

    const cv::Vec3d vi = v1 - v0;
    const cv::Vec3d vj = v2 - v1;
    cv::Vec3d z;
    cv::normalize( vi.cross(vj), z);

    // The triangle area used is the ORIGINAL area of the triangle (because unfolding can warp the area)
    const double A = calcTriangleArea( _model->vtx(vidxs[0]), _model->vtx(vidxs[1]), _model->vtx(vidxs[2]));

    return v0 + pp[0]*(v1-v0) + pp[1]*(v2-v1) + pp[2]*sqrt(2*A)*z;
}   // end calcUnfoldedPoint


void ObjModelPolyUnfolder::reset( int T)
{
    _uvtxs.clear();
    _upolys.clear();
    _pnorm = cv::Vec3d(0,0,0);
    if ( _model->getFaceIds().count(T) == 0)
        return;

    const int* vidxs = _model->getFaceVertices(T);
    assert(vidxs);
    const int r = vidxs[0];
    const int a = vidxs[1];
    const int b = vidxs[2];
    _uvtxs[r] = _model->vtx(r);
    _uvtxs[a] = _model->vtx(a);
    _uvtxs[b] = _model->vtx(b);
    _upolys.insert(T);
    _pnorm = _model->calcFaceNorm(T);
}   // end reset


int ObjModelPolyUnfolder::unfold( int T, int r, int a)
{
    if ( _model->getFaceIds().count(T) == 0)
        return -1;

    const ObjPoly& face = _model->getFace( T);
    const int b = face.opposite( r, a);

    if ( _uvtxs.count(r) == 0 || _uvtxs.count(a) == 0)
    {
        reset( T);
        return b;
    }   // end if

    face.opposite( b, r, a);    // Ensure correct order of a,r
    const cv::Vec3d& va = _uvtxs.at(a);
    const cv::Vec3d& vr = _uvtxs.at(r);
    cv::Vec3d u, v;
    cv::normalize( va - vr, u);         // Unit vector unfolding edge
    cv::normalize( _pnorm.cross(u), v); // Unit vector orthogonal to plane and folding edge (sign to be determined)

    const cv::Vec3d br = _model->vtx(b) - _model->vtx(r);
    cv::Vec3d ou;
    cv::normalize( _model->vtx(a) - _model->vtx(r), ou);
    const double m = br.dot(ou);    // Amount along u
    const double n = cv::norm( br - m*ou);  // Amount along v

    _uvtxs[b] = vr + m*u + n*v;
    _upolys.insert(T);

    return b;
}   // end unfold


void ObjModelPolyUnfolder::unfoldPath( const std::vector<int>& spvids, int T, int fT)
{
    assert( spvids.size() >= 1);
    assert( _model->getFaceIds(spvids[0]).count(T) > 0);
    assert( _model->getFaceIds(*spvids.rbegin()).count(fT) > 0);
    reset();

    std::vector<int>::const_iterator it = spvids.begin();
    // a,b are consecutive point pair vertex indices in spvids.
    int a = *it++;
    int b = it != spvids.end() ? *it : -1;
    int x; // Don't care which edge of T to start on
    _model->poly(T).getOpposite( a, x, x);

    while ( true)
    {
        x = unfold(T, a, x);
        if ( T == fT)
            break;

        int nT = oppositePoly( _model, T, a, x);
        if ( nT >= 0)   // No need to do anything else if nT < 0: will go back the other way!
            T = nT;

        if ( x == b)    // Concertina around the other way!
        {
            x = a;
            a = *it++;
            b = it != spvids.end() ? *it : -1;
        }   // end if
    }   // end while
}   // unfoldPath
