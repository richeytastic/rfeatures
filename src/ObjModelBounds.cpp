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

#include <ObjModelBounds.h>
#include <FeatureUtils.h>
using RFeatures::ObjModelBounds;
using RFeatures::ObjModel;
using RFeatures::ObjPoly;

ObjModelBounds::Ptr ObjModelBounds::create( const cv::Vec3d& minc, const cv::Vec3d& maxc, const cv::Matx44d& m)
{
    return Ptr( new ObjModelBounds( minc, maxc, m));
}   // end create


ObjModelBounds::Ptr ObjModelBounds::create( const ObjModel& model, const cv::Matx44d& m, const IntSet* vset)
{
    return Ptr( new ObjModelBounds( model, m, vset));
}   // end create


ObjModelBounds::Ptr ObjModelBounds::create( const ObjModel& model, const cv::Matx44d& m, const IntSet& pset)
{
    return Ptr( new ObjModelBounds( model, m, pset));
}   // end create


void ObjModelBounds::_init( const ObjModel& model, const cv::Matx44d& m, const IntSet& vidxs)
{
    _minc[0] = _minc[1] = _minc[2] = DBL_MAX;
    _maxc[0] = _maxc[1] = _maxc[2] = -DBL_MAX;

    _tmat = m;
    _imat = m.inv();

    for ( int vidx : vidxs)
    {
        const cv::Vec3f v = RFeatures::transform( _imat, model.vtx(vidx));

        const double x = v[0];
        if ( x < _minc[0])
            _minc[0] = x;
        if ( x > _maxc[0])
            _maxc[0] = x;

        const double y = v[1];
        if ( y < _minc[1])
            _minc[1] = y;
        if ( y > _maxc[1])
            _maxc[1] = y;

        const double z = v[2];
        if ( z < _minc[2])
            _minc[2] = z;
        if ( z > _maxc[2])
            _maxc[2] = z;
    }   // end for

    _calcExtents();
}   // end _init


void ObjModelBounds::_calcExtents()
{
    cv::Vec3d ux, uy, uz;   // Get the unit vectors along the edges of the cuboid in three directions
    cv::normalize( cv::Vec3d(_tmat(0,0), _tmat(1,0), _tmat(2,0)), ux);
    cv::normalize( cv::Vec3d(_tmat(0,1), _tmat(1,1), _tmat(2,1)), uy);
    cv::normalize( cv::Vec3d(_tmat(0,2), _tmat(1,2), _tmat(2,2)), uz);

    const double dx = xlen();
    const double dy = ylen();
    const double dz = zlen();

    const cv::Vec3d minc = minCorner(); // Transformed
    const cv::Vec3d maxc = maxCorner(); // Transformed

    // Get the other possible corners in the transformed space:
    const cv::Vec3d c0 = minc + ux*dx;
    const cv::Vec3d c1 = minc + uy*dy;
    const cv::Vec3d c2 = minc + uz*dz;
    const cv::Vec3d c3 = maxc - ux*dx;
    const cv::Vec3d c4 = maxc - uy*dy;
    const cv::Vec3d c5 = maxc - uz*dz;

    _mine = cv::Vec3d( std::min(minc[0], std::min(c0[0], std::min(c1[0], std::min(c2[0], std::min(c3[0], std::min(c4[0], std::min(c5[0], maxc[0]))))))),
                       std::min(minc[1], std::min(c0[1], std::min(c1[1], std::min(c2[1], std::min(c3[1], std::min(c4[1], std::min(c5[1], maxc[1]))))))),
                       std::min(minc[2], std::min(c0[2], std::min(c1[2], std::min(c2[2], std::min(c3[2], std::min(c4[2], std::min(c5[2], maxc[2]))))))));

    _maxe = cv::Vec3d( std::max(minc[0], std::max(c0[0], std::max(c1[0], std::max(c2[0], std::max(c3[0], std::max(c4[0], std::max(c5[0], maxc[0]))))))),
                       std::max(minc[1], std::max(c0[1], std::max(c1[1], std::max(c2[1], std::max(c3[1], std::max(c4[1], std::max(c5[1], maxc[1]))))))),
                       std::max(minc[2], std::max(c0[2], std::max(c1[2], std::max(c2[2], std::max(c3[2], std::max(c4[2], std::max(c5[2], maxc[2]))))))));
}   // end _calcExtents


ObjModelBounds::ObjModelBounds( const cv::Vec3d& minc, const cv::Vec3d& maxc, const cv::Matx44d& m)
{
    _tmat = m;
    _imat = m.inv();

    _minc = RFeatures::transform( _imat, minc);
    _maxc = RFeatures::transform( _imat, maxc);
    std::cerr << "Initial bounds: " << std::endl;
    std::cerr << _minc << std::endl;
    std::cerr << _maxc << std::endl;

    _calcExtents();
}   // end ctor


ObjModelBounds::ObjModelBounds( const ObjModel& model, const cv::Matx44d& m, const IntSet* vset)
{
    if ( !vset)
        vset = &model.vtxIds();
    _init( model, m, *vset);
}   // end ctor


ObjModelBounds::ObjModelBounds( const ObjModel& model, const cv::Matx44d& m, const IntSet& pset)
{
    IntSet vset;
    for ( int fid : pset)
    {
        const ObjPoly& p = model.face(fid);
        vset.insert(p[0]);
        vset.insert(p[1]);
        vset.insert(p[2]);
    }   // end for
    _init( model, m, vset);
}   // end ctor


ObjModelBounds::Ptr ObjModelBounds::deepCopy() const
{
    return Ptr( new ObjModelBounds(*this));
}   // end deepCopy


void ObjModelBounds::setTransformMatrix( const cv::Matx44d& tmat)
{
    _tmat = tmat;
    _imat = tmat.inv();
    _calcExtents();
}   // end setTransformMatrix


void ObjModelBounds::addTransformMatrix( const cv::Matx44d& tmat)
{
    setTransformMatrix( tmat * _tmat);
}   // end addTransformMatrix


void ObjModelBounds::fixTransformMatrix()
{
    // The extents will be the same as the new corners given that we're resetting to the identity matrix.
    _mine = _minc = minCorner();
    _maxe = _maxc = maxCorner();
    _tmat = _imat = cv::Matx44d::eye();
}   // end fixTransformMatrix


void ObjModelBounds::encompass( const ObjModelBounds& ob)
{
    ObjModelBounds tob = ob;
    tob.addTransformMatrix( _imat);
    const cv::Vec3d& min1c = tob.minExtent();
    const cv::Vec3d& max1c = tob.maxExtent();

    // Update
    _minc[0] = std::min( _minc[0], min1c[0]);
    _minc[1] = std::min( _minc[1], min1c[1]);
    _minc[2] = std::min( _minc[2], min1c[2]);

    _maxc[0] = std::max( _maxc[0], max1c[0]);
    _maxc[1] = std::max( _maxc[1], max1c[1]);
    _maxc[2] = std::max( _maxc[2], max1c[2]);

    _calcExtents();
}   // end encompass


bool ObjModelBounds::intersects( const ObjModelBounds& ob) const
{
    // Transform the parameter bounds into standard position with respect to these bounds.
    ObjModelBounds tob = ob;
    tob.addTransformMatrix( _imat);
    const cv::Vec3d& min1c = ob.minExtent();
    const cv::Vec3d& max1c = ob.maxExtent();

    const cv::Vec6d a( _minc[0], _maxc[0], _minc[1], _maxc[1], _minc[2], _maxc[2]);
    const cv::Vec6d b( min1c[0], max1c[0], min1c[1], max1c[1], min1c[2], max1c[2]);

    const bool xAinB = ((a[0] >= b[0] && a[0] <= b[1]) || (a[1] >= b[0] && a[1] <= b[1]));    // x edges of A in B
    const bool xBinA = ((b[0] >= a[0] && b[0] <= a[1]) || (b[1] >= a[0] && b[1] <= a[1]));    // x edges of B in A
    const bool yAinB = ((a[2] >= b[2] && a[2] <= b[3]) || (a[3] >= b[2] && a[3] <= b[3]));    // y edges of A in B
    const bool yBinA = ((b[2] >= a[2] && b[2] <= a[3]) || (b[3] >= a[2] && b[3] <= a[3]));    // y edges of B in A
    const bool zAinB = ((a[4] >= b[4] && a[4] <= b[5]) || (a[5] >= b[4] && a[5] <= b[5]));    // y edges of A in B
    const bool zBinA = ((b[4] >= a[4] && b[4] <= a[5]) || (b[5] >= a[4] && b[5] <= a[5]));    // y edges of B in A

    const bool xint = xAinB || xBinA;
    const bool yint = yAinB || yBinA;
    const bool zint = zAinB || zBinA;

    return xint && yint && zint;
}   // end intersects


cv::Vec3d ObjModelBounds::minCorner() const { return RFeatures::transform( _tmat, _minc);}
cv::Vec3d ObjModelBounds::maxCorner() const { return RFeatures::transform( _tmat, _maxc);}
cv::Vec3d ObjModelBounds::centre() const { return RFeatures::transform( _tmat, (_minc + _maxc) * 0.5);}


double ObjModelBounds::xlen() const
{
    // Get sX as the scale factor along the x axis.
    const cv::Vec3d xvec(_tmat(0,0), _tmat(1,0), _tmat(2,0));
    const double sX = cv::norm( xvec);
    const double d = fabs(_minc[0] - _maxc[0]);
    return sX * d;
}   // end xlen


double ObjModelBounds::ylen() const
{
    // Get sY as the scale factor along the y axis.
    const double sY = cv::norm( cv::Vec3d(_tmat(0,1), _tmat(1,1), _tmat(2,1)));
    return sY * fabs(_minc[1] - _maxc[1]);
}   // end ylen


double ObjModelBounds::zlen() const
{
    // Get sZ as the scale factor along the z axis.
    const double sZ = cv::norm( cv::Vec3d(_tmat(0,2), _tmat(1,2), _tmat(2,2)));
    return sZ * fabs(_minc[2] - _maxc[2]);
}   // end zlen


double ObjModelBounds::diagonal() const
{
    const cv::Vec3d minc = minCorner();
    const cv::Vec3d maxc = maxCorner();
    return cv::norm( minc, maxc);
}   // end diagonal


// Note that the raw non-transformed vertices are being used here!
cv::Vec6d ObjModelBounds::cornersAs6d() const { return as6d(_minc, _maxc);}

cv::Vec6d ObjModelBounds::extentsAs6d() const { return as6d(_mine, _maxe);}


// static
cv::Vec6d ObjModelBounds::as6d( const cv::Vec3d& a, const cv::Vec3d& b)
{
    cv::Vec6d rv;

    if ( a[0] < b[0])
    {
        rv[0] = a[0];
        rv[1] = b[0];
    }   // end if
    else
    {
        rv[0] = b[0];
        rv[1] = a[0];
    }   // end else

    if ( a[1] < b[1])
    {
        rv[2] = a[1];
        rv[3] = b[1];
    }   // end if
    else
    {
        rv[2] = b[1];
        rv[3] = a[1];
    }   // end else

    if ( a[2] < b[2])
    {
        rv[4] = a[2];
        rv[5] = b[2];
    }   // end if
    else
    {
        rv[4] = b[2];
        rv[5] = a[2];
    }   // end else

    return rv;
}   // end as6d
