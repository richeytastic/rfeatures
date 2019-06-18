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

#include <Transformer.h>
#include <FeatureUtils.h>
using RFeatures::Transformer;
using RFeatures::ObjModel;

Transformer::Transformer()
    : _tmat( 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1)
{}   // end ctor

Transformer::Transformer( const cv::Matx44d& t) : _tmat(t) {}

Transformer::Transformer( const cv::Vec3d& t)
    : _tmat( 1,  0,  0, t[0],
             0,  1,  0, t[1],
             0,  0,  1, t[2],
             0,  0,  0,  1)
{}   // end ctor

Transformer::Transformer( const cv::Vec3f& t)
    : _tmat( 1,  0,  0, t[0],
             0,  1,  0, t[1],
             0,  0,  1, t[2],
             0,  0,  0,  1)
{}   // end ctor

Transformer::Transformer( const cv::Matx33d& R, const cv::Vec3d& t)
    : _tmat( R(0,0), R(0,1), R(0,2), t[0],
             R(1,0), R(1,1), R(1,2), t[1],
             R(2,0), R(2,1), R(2,2), t[2],
                0,      0,      0,    1)
{}   // end ctor


Transformer::Transformer( const cv::Vec3d& vnorm, const cv::Vec3d& vup, const cv::Vec3d& t)
    : _tmat( 1,  0,  0, t[0],
             0,  1,  0, t[1],
             0,  0,  1, t[2],
             0,  0,  0,  1)
{
    // Ensure normalized
    cv::Vec3d uvec, zvec;
    cv::normalize( vup, uvec);
    cv::normalize( vnorm, zvec);

    cv::Vec3d xvec = uvec.cross(zvec);
    cv::Vec3d yvec = zvec.cross(xvec);   // Don't trust that vup and vnorm actually are orthogonal

    _tmat(0,0) = xvec[0];
    _tmat(0,1) = yvec[0];
    _tmat(0,2) = zvec[0];

    _tmat(1,0) = xvec[1];
    _tmat(1,1) = yvec[1];
    _tmat(1,2) = zvec[1];

    _tmat(2,0) = xvec[2];
    _tmat(2,1) = yvec[2];
    _tmat(2,2) = zvec[2];
}   // end ctor



namespace {

void _initR( double rads, const cv::Vec3d& axis, cv::Matx44d& tmat)
{
    cv::Vec3d u;    // Ensure normalised axis
    cv::normalize( axis, u);
    const double x = u[0];
    const double y = u[1];
    const double z = u[2];
    const double ct = cos(rads);
    const double mct = 1.0-ct;
    const double st = sin(rads);
    const double xst = x*st;
    const double yst = y*st;
    const double zst = z*st;

    // Set the 3x3 upper left submatrix with the rotation params
    tmat(0,0) = x*x*mct + ct;
    tmat(0,1) = x*y*mct - zst;
    tmat(0,2) = x*z*mct + yst;

    tmat(1,0) = y*x*mct + zst;
    tmat(1,1) = y*y*mct + ct;
    tmat(1,2) = y*z*mct - xst;

    tmat(2,0) = z*x*mct - yst;
    tmat(2,1) = z*y*mct + xst;
    tmat(2,2) = z*z*mct + ct;
}   // end _initR

}   // end namespace


Transformer::Transformer( double rads, const cv::Vec3d& axis, const cv::Vec3d& t)
    : _tmat( 1,  0,  0, t[0],
             0,  1,  0, t[1],
             0,  0,  1, t[2],
             0,  0,  0,   1)
{
    _initR( rads, axis, _tmat);
}   // end ctor


Transformer::Transformer( double rads, const cv::Vec3f& fa, const cv::Vec3f& t)
    : _tmat( 1,  0,  0, t[0],
             0,  1,  0, t[1],
             0,  0,  1, t[2],
             0,  0,  0,   1)
{
    _initR( rads, cv::Vec3d(fa[0], fa[1], fa[2]), _tmat);
}   // end ctor


void Transformer::prependTranslation( const cv::Vec3d& t)
{
    const cv::Matx44d T( 1, 0, 0, t[0],
                         0, 1, 0, t[1],
                         0, 0, 1, t[2],
                         0, 0, 0, 1);
    _tmat = _tmat * T;  // Note that this prepends because applied transform will be (_tmat * T * v) for some vector v
}   // end prependTranslation


Transformer& Transformer::operator*( const Transformer& m)
{
    _tmat = m.matrix() * _tmat;
    return *this;
}   // end operator*


void Transformer::transform( cv::Vec3d& v) const
{
    const cv::Vec4d nv = _tmat * cv::Vec4d( v[0], v[1], v[2], 1);
    v[0] = nv[0];
    v[1] = nv[1];
    v[2] = nv[2];
}   // end transform


void Transformer::transform( cv::Vec3f& v) const
{
    const cv::Vec4d nv = _tmat * cv::Vec4d( v[0], v[1], v[2], 1);
    v[0] = (float)nv[0];
    v[1] = (float)nv[1];
    v[2] = (float)nv[2];
}   // end transform


cv::Vec3d Transformer::transform( const cv::Vec3d& v) const
{
    cv::Vec3d vn = v;
    transform(vn);
    return vn;
}   // end transform


cv::Vec3f Transformer::transform( const cv::Vec3f& v) const
{
    cv::Vec3f vn = v;
    transform(vn);
    return vn;
}   // end transform


void Transformer::rotate( cv::Vec3d& v) const
{
    const cv::Matx33d rmat = cv::Mat( _tmat)( cv::Range(0,3), cv::Range(0,3));
    v = rmat * v;
}   // end rotate


void Transformer::rotate( cv::Vec3f& v) const
{
    cv::Vec3d nv = v;
    rotate(nv);
    v[0] = (float)nv[0];
    v[1] = (float)nv[1];
    v[2] = (float)nv[2];
}   // end rotate


cv::Vec3f Transformer::rotate( const cv::Vec3f& v) const
{
    cv::Vec3f vn = v;
    rotate(vn);
    return vn;
}   // end rotate


cv::Vec3d Transformer::rotate( const cv::Vec3d& v) const
{
    cv::Vec3d vn = v;
    rotate(vn);
    return vn;
}   // end rotate


/*
void Transformer::transform( ObjModel::Ptr model) const
{
    cv::Mat vtxs = cv::Mat(model->vertices().t()).reshape(1,3);   // Get vertices as columns and reshape to be single channel
    assert( vtxs.rows == 3);
    const int nv = vtxs.cols;
    assert( nv == model->numVertices());
    vtxs.push_back( cv::Mat::ones(1, vtxs.cols, CV_32FC1));   // Make homogenous
    assert( vtxs.rows == 4);
    cv::Mat tmat;   // Convert the transformation matrix to the required depth
    cv::Mat(_tmat).convertTo( tmat, CV_32F);
    assert( tmat.cols == 4);
    assert( tmat.rows == 4);
 
    // Do the transformation and set back.
    cv::Mat o = tmat * vtxs;
    assert( o.rows == 4);
    assert( o.cols == nv);
    cv::Mat ot = o.rowRange(0,3);
    assert( ot.rows == 3);
    assert( ot.cols == nv);
    cv::Mat ft = ot.reshape(3,1).t();   // Reshape back to 3 channel single row.

    model->vertices() = ft; // Set back in the model
}   // end transform
*/


void Transformer::transform( ObjModel::Ptr model) const
{
    const IntSet& vidxs = model->vtxIds();
    for ( int i : vidxs)
        model->adjustVertex( i, transform( model->vtx(i)));
}   // end transform


// public
cv::Matx44d RFeatures::toStandardPosition( const cv::Vec3f& vnrm, const cv::Vec3f& vup, const cv::Vec3f& mpos)
{
    cv::Vec3d upv, nrm;
    cv::normalize( vnrm, nrm);
    cv::normalize( vup, upv);

    static const cv::Vec3d ZPOS(0,0,1); // Standard position for normal (+Z)
    static const cv::Vec3d YPOS(0,1,0); // Standard position for up vector (+Y)

    // First get the angle difference between the current normal and ZPOS
    double v = std::min<double>( std::max<double>( -1, nrm.dot(ZPOS)), 1);
    const double zrads = acos( v);
    Transformer rot2z;  // Identity

    if ( zrads > 0)
    {
        // Rotate with positive or negative degrees that causes nrm to be incident with ZPOS
        const cv::Vec3d caxis = nrm.cross(ZPOS);
        rot2z = Transformer( zrads, caxis);
        rot2z.transform( nrm);
        v = std::min<double>( std::max<double>( -1, nrm.dot(ZPOS)), 1);
        if ( acos( v) > zrads)
            rot2z = Transformer( -zrads, caxis);
    }   // end if

    rot2z.transform( upv);   // Rotate the orientation up-vector, then measure the angle difference with YPOS

    v = std::min<double>( std::max<double>( -1, upv.dot(YPOS)), 1);
    const double yrads = acos( v);
    Transformer rot2y;  // Identity
   
    if ( yrads > 0) 
    {
        rot2y = Transformer( yrads, ZPOS); // Assumes orientation normal is now incident with ZPOS so next rotation is about ZPOS
        rot2y.transform( upv);
        v = std::min<double>( std::max<double>( -1, upv.dot(YPOS)), 1);
        if ( acos( v) > yrads)
            rot2y = Transformer( -yrads, ZPOS);
    }   // end if

    Transformer tmat( -mpos);
    return rot2y() * rot2z() * tmat();
}   // end toStandardPosition
