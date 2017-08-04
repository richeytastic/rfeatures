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

#include "ObjModelAligner.h"
using RFeatures::ObjModelAligner;
using RFeatures::ObjModel;
#include <icpPointToPlane.h>    // Andreas Geiger
#include <boost/foreach.hpp>
#include <cstring>
#include <cassert>
#include <vector>


cv::Vec3d getMeanPosition( const ObjModel::Ptr model)
{
    cv::Vec3d spos(0,0,0);
    const IntSet& uvidxs = model->getVertexIds();
    BOOST_FOREACH ( const int& uvidx, uvidxs)
    {
        const cv::Vec3f& v = model->getVertex(uvidx);
        spos[0] += v[0];
        spos[1] += v[1];
        spos[2] += v[2];
    }   // end foreach
    return spos * (1./ model->getNumVertices());
}   // end getMeanPosition


void setVertex( double* mpoints, const cv::Vec3f& v)
{
    mpoints[0] = v[0];
    mpoints[1] = v[1];
    mpoints[2] = v[2];
}   // end setVertex


double* createModelPointsArray( const ObjModel::Ptr model, const std::vector<int>* uvs=NULL)
{
    const int N = (int)model->getNumVertices();
    double* mpoints = new double[3*N];
    const IntSet& uvidxs = model->getVertexIds();
    if ( uvs)
    {
        assert( (int)uvs->size() == N);
        for ( int k = 0; k < N; ++k)
            setVertex( &mpoints[k*3], model->getVertex(uvs->at(k)));
    }   // end if
    else
    {
        int k = 0;
        BOOST_FOREACH ( const int& uvidx, uvidxs)
        {
            setVertex( &mpoints[k*3], model->getVertex(uvidx));
            k++;
        }   // end foreach
    }   // end foreach
    return mpoints;
}   // end createModelPointsArray


void createVertexIndexVector( const ObjModel::Ptr model, std::vector<int>& uvmap)
{
    const int N = (int)model->getNumVertices();
    uvmap.resize(N);
    const IntSet& uvidxs = model->getVertexIds();
    int k = 0;
    BOOST_FOREACH ( const int& uvidx, uvidxs)
        uvmap[k++] = uvidx;
}   // end createVertexIndexVector


// public static
ObjModelAligner::Ptr ObjModelAligner::create( const ObjModel::Ptr m)
{
    return Ptr( new ObjModelAligner(m));
}   // end create


// public
ObjModelAligner::ObjModelAligner( const ObjModel::Ptr m) : _model(m)
{
    assert( m->getNumVertices() >= 5);
    _T = createModelPointsArray( m);
}   // end ctor


// public
ObjModelAligner::~ObjModelAligner()
{
    delete[] _T;
}   // end dtor


// public
cv::Matx44d ObjModelAligner::calcTransform( const ObjModel::Ptr model) const
{
    static const int32_t NDIMS = 3;
    IcpPointToPlane icp( _T, (int32_t)_model->getNumVertices(), NDIMS);

    std::vector<int> uvmap;
    createVertexIndexVector( model, uvmap);
    double* M = createModelPointsArray( model, &uvmap);
    const int N = (int)uvmap.size();
    //checkArray( M, model, uvmap);

    Matrix R = Matrix::eye(3);  // Identity matrix as initial rotation matrix
    Matrix t(3,1);
    icp.fit( M, N, R, t, -1/*use all points*/);
    delete[] M;

    return cv::Matx44d( R.val[0][0], R.val[0][1], R.val[0][2], t.val[0][0],
                        R.val[1][0], R.val[1][1], R.val[1][2], t.val[1][0],
                        R.val[2][0], R.val[2][1], R.val[2][2], t.val[2][0],
                                  0,           0,           0,           1);
}   // end calcTransform


cv::Matx44d ObjModelAligner::operator()( const ObjModel::Ptr model) const
{
    return calcTransform( model);
}   // end operator()

