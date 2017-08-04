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

#include "ObjModelOrienter.h"
using RFeatures::ObjModelOrienter;
using RFeatures::ObjModel;
#include <boost/foreach.hpp>
#include <iostream>
#include <Eigen/Dense>


// public
ObjModelOrienter::ObjModelOrienter( ObjModel::Ptr m) : _model(m)
{
}   // end ctor


void updateModelWithNewPoints( ObjModel::Ptr m, const std::vector<int>& uvMap, const Eigen::MatrixXd P, const cv::Vec3f mu)
{
    const int N = (int)uvMap.size();
    for ( int i = 0; i < N; ++i)
    {
        const Eigen::Vector3d& v = P.col(i);
        m->adjustVertex( uvMap[i], float(v[0]) + mu[0], float(v[1]) + mu[1], float(v[2]) + mu[2]);
    }   // end for
}   // end updateModelWithNewPoints


Eigen::Matrix3d createCovariance( const Eigen::MatrixXd& P)
{
    const int N = (int)P.cols();
    Eigen::Matrix3d C = Eigen::Matrix3d::Zero(3,3);
    for ( int i = 0; i < N; ++i)
        C += P.col(i) * P.col(i).transpose();
    C /= N;
    return C;
}   // end createCovariance


// public
cv::Vec3f ObjModelOrienter::orientPCA()
{
    // Map the unique vertex indices into a consistent array
    const IntSet& uvidxs = _model->getVertexIds();
    const int N = (int)uvidxs.size();
    Eigen::MatrixXd P = Eigen::MatrixXd( 3, N);
    std::vector<int> uvMap( N);
    int i = 0;
    // Create the initial points matrix (one column per vertex)
    BOOST_FOREACH ( const int& uvidx, uvidxs)
    {
        const cv::Vec3f& v = _model->getVertex( uvidx);
        P( 0, i) = v[0];
        P( 1, i) = v[1];
        P( 2, i) = v[2];
        uvMap[i++] = uvidx;
    }   // end foreach

    const Eigen::Vector3d m = P.rowwise().sum() / N;        // Mean position
    const cv::Vec3f mv( (float)m[0], (float)m[1], (float)m[2]);
    P = P.colwise() - m;    // Transform to origin

    Eigen::Matrix3d I = Eigen::Matrix3d::Identity(3,3);
    Eigen::Matrix3d V = I;
    do
    {
        Eigen::Matrix3d C = createCovariance( P);
        Eigen::EigenSolver<Eigen::Matrix3d> eig(C, true);
        V = eig.eigenvectors().real();
        /*
        if ( V(0,0) < 0)
            I(0,0) *= -1;
        if ( V(1,1) < 0)
            I(1,1) *= -1;
        if ( V(2,2) < 0)
            I(2,2) *= -1;
        P = V*I*P;    // Update positions of points
        */
        P = V*P;
    } while ( fabs((V - I).sum()) > 0.0000001);

    //std::cerr << "Eigenvectors:" << std::endl;
    //std::cerr << V << std::endl;
    //std::cerr << "Eigenvalues: " << eig.eigenvalues().transpose().real() << std::endl;
    updateModelWithNewPoints( _model, uvMap, P, mv);
    return mv;
}   // end orientPCA
