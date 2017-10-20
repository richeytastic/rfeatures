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

#include <ObjModelAligner.h>
using RFeatures::ObjModelAligner;
using RFeatures::ObjModel;
#include <icpPointToPlane.h>    // Andreas Geiger
#include <boost/foreach.hpp>
#include <cstring>
#include <cassert>
#include <vector>

void setVertex( double* mpoints, const cv::Vec3f& v)
{
    mpoints[0] = v[0];
    mpoints[1] = v[1];
    mpoints[2] = v[2];
}   // end setVertex


double* createModelPointsArray( const ObjModel::Ptr model, int& N)
{
    N = (int)model->getNumVertices();
    assert( N >= 5);

    double* mpoints = new double[3*N];
    int k = 0;
    const IntSet& vidxs = model->getVertexIds();
    BOOST_FOREACH ( int vidx, vidxs)
    {
        setVertex( &mpoints[k*3], model->vtx(vidx));
        k++;
    }   // end foreach
    return mpoints;
}   // end createModelPointsArray


// public static
ObjModelAligner::Ptr ObjModelAligner::create( const ObjModel::Ptr m)
{
    return Ptr( new ObjModelAligner(m));
}   // end create


ObjModelAligner::ObjModelAligner( const ObjModel::Ptr m)
{
    _T = createModelPointsArray( m, _n);
}   // end ctor


ObjModelAligner::~ObjModelAligner()
{
    delete[] _T;
}   // end dtor


cv::Matx44d ObjModelAligner::calcTransform( const ObjModel::Ptr model) const
{
    static const int32_t NDIMS = 3;
    IcpPointToPlane icp( _T, _n, NDIMS);

    int N;
    double* M = createModelPointsArray( model, N);
    Matrix R = Matrix::eye(3);  // Identity matrix as initial rotation matrix
    Matrix t(3,1);
    icp.fit( M, N, R, t, -1/*use all points*/);
    delete[] M;

    return cv::Matx44d( R.val[0][0], R.val[0][1], R.val[0][2], t.val[0][0],
                        R.val[1][0], R.val[1][1], R.val[1][2], t.val[1][0],
                        R.val[2][0], R.val[2][1], R.val[2][2], t.val[2][0],
                                  0,           0,           0,           1);
}   // end calcTransform
