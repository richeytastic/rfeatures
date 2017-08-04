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

#include "ObjModelFunctionMapper.h"
using RFeatures::ObjModelFunctionMapper;
using RFeatures::ObjModel;
#include <iostream>
#include <iomanip>


// public
ObjModelFunctionMapper::ObjModelFunctionMapper( int rows, int cols, double xo, double yo, double xs, double ys)
    : _zvals( rows, cols), _useLiteral(false), _vidxs( rows, cols), _xoffset(xo), _yoffset(yo), _xscale(xs), _yscale(ys)
{
}   // end ctor


// public
ObjModelFunctionMapper::ObjModelFunctionMapper( const cv::Mat_<double>& litZ)
    : _zvals(litZ), _useLiteral(true), _vidxs( litZ.rows, litZ.cols), _xoffset(0), _yoffset(0), _xscale(0), _yscale(0)
{
}   // end ctor


// public
ObjModelFunctionMapper::~ObjModelFunctionMapper()
{
}   // end dtor


// public
bool ObjModelFunctionMapper::textureMap( const std::string& txfile)
{
    const cv::Mat m = cv::imread( txfile);
    return textureMap( m);
}   // end textureMap


// public
bool ObjModelFunctionMapper::textureMap( const cv::Mat& m)
{
    if ( m.empty() || !_model)
        return false;

    _model->removeAllMaterials();
    const int mid = _model->addMaterial();
    _model->addMaterialDiffuse( mid, m);

    const int nrows = _vidxs.rows;
    const int ncols = _vidxs.cols;

    const float txstep = 1.0f/ncols;
    const float tystep = 1.0f/nrows;

    int fid = 0;
    for ( int i = 1; i < nrows; ++i)
    {
        for ( int j = 1; j < ncols; ++j)
        {
            const cv::Vec2f uv0( tystep*i, txstep*j);
            const cv::Vec2f uv1( uv0[0], txstep*(j-1));
            const cv::Vec2f uv2( tystep*(i-1), uv1[1]);
            const cv::Vec2f uv3( uv2[0], uv0[1]);

            const int vdxsA[3] = { _vidxs(i,j), _vidxs(i,j-1), _vidxs(i-1,j-1)};
            const cv::Vec2f uvsA[3] = { uv0, uv1, uv2};
            _model->setOrderedFaceTextureOffsets( mid, fid++, vdxsA, uvsA);

            const int vdxsB[3] = { _vidxs(i,j), _vidxs(i-1,j), _vidxs(i-1,j-1)};
            const cv::Vec2f uvsB[3] = { uv0, uv3, uv2};
            _model->setOrderedFaceTextureOffsets( mid, fid++, vdxsB, uvsB);
        }   // end for
    }   // end for

    return true;
}   // end textureMap


// public
ObjModel::Ptr ObjModelFunctionMapper::operator()()
{
    _model = ObjModel::create();
    const int nrows = _zvals.rows;
    const int ncols = _zvals.cols;

    const bool mapLiteralZ = _useLiteral;
    double x, y, z;
    const double xoffset = _xoffset;
    const double yoffset = _yoffset;
    const double xstep = _xscale/ncols;
    const double ystep = _yscale/nrows;
    const int m = -nrows/2;
    const int n = -ncols/2;

    _vidxs = cv::Mat_<int>( nrows, ncols);

    for ( int i = 0; i < nrows; ++i)
    {
        y = ystep*(m+i) + yoffset;  // Only used if mapping literal z vals

        for ( int j = 0; j < ncols; ++j)
        {
            if ( mapLiteralZ)
                _vidxs(i,j) = _model->addVertex( double(j), double(i), _zvals.at<double>(i,j));
            else
            {
                x = xstep*(n+j) + xoffset;
                z = _zvals.at<double>(i,j) = calcZ( x, y);
                _vidxs(i,j) = _model->addVertex( x, y, z);
                //std::cerr << "fn( " << std::setw(6) << std::fixed << std::setprecision(4) << x
                //            << ", " << std::setw(6) << std::fixed << std::setprecision(4) << y
                //            << ") = " << std::fixed << std::setprecision(4) << z << std::endl;
            }   // end else

            if ( i > 0 && j > 0)
            {
                _model->setFace( _vidxs(i,j), _vidxs(i,j-1), _vidxs(i-1,j-1));
                _model->setFace( _vidxs(i,j), _vidxs(i-1,j), _vidxs(i-1,j-1));
            }   // end if
        }   // end for
    }   // end for

    return _model;
}   // end operator()


// protected virtual
double ObjModelFunctionMapper::calcZ( double x, double y) const
{
    return 0.0;
}   // end calcZ
