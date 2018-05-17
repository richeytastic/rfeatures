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

#include <ObjModelPatchBendingEnergy.h>
using RFeatures::ObjModelPatchBendingEnergy;
using RFeatures::ObjModel;
#include <Eigen/Dense>
#include <cassert>
#include <cmath>


// public
ObjModelPatchBendingEnergy::ObjModelPatchBendingEnergy( const ObjModel::Ptr m0, const ObjModel::Ptr m1)
    : _m0(m0), _m1(m1)
{
}   // end ctor


int setCoordinateVectors( const ObjModel::Ptr model, const IntSet& vset,
                          Eigen::VectorXd& x, Eigen::VectorXd& y, Eigen::VectorXd& z)
{
    const int m = (int)vset.size();
    x.resize(m);
    y.resize(m);
    z.resize(m);
    int i = 0;
    for ( int vidx : vset)
    {
        const cv::Vec3f& v = model->getVertex(vidx);
        x[i] = v[0];
        y[i] = v[1];
        z[i] = v[2];
        i++;
    }   // end foreach
    return m;
}   // end setCoordinateVectors


int setCoordinateVectors( const cv::Mat_<cv::Vec3f>& vset, Eigen::VectorXd& x, Eigen::VectorXd& y, Eigen::VectorXd& z)
{
    const int m = (int)vset.rows;
    x.resize(m);
    y.resize(m);
    z.resize(m);
    for ( int i = 0; i < m; ++i)
    {
        const cv::Vec3f& v = vset.ptr<cv::Vec3f>(i)[0];
        x[i] = v[0];
        y[i] = v[1];
        z[i] = v[2];
    }   // end for
    return m;
}   // end setCoordinateVectors


Eigen::MatrixXd setOmega( const Eigen::VectorXd& x, const Eigen::VectorXd& y, const Eigen::VectorXd& z)
{
    const int m = (int)x.size();
    assert( m == (int)y.size() && m == (int)z.size());

    Eigen::MatrixXd B( m, 4);
    B << Eigen::VectorXd::Ones(m), x, y, z; // Eigen is column major
    Eigen::MatrixXd Bt = B.transpose();

    // Create matrix K
    Eigen::MatrixXd K( m, m);
    for ( int i = 0; i < m; ++i)
    {
        for ( int j = 0; j < m; ++j)
        {
            const double sqDiff = pow(x[i]-x[j],2) + pow(y[i]-y[j],2) + pow(z[i]-z[j],2);
            K(i,j) = sqDiff > 0.0 ? sqDiff * log( sqDiff) : 0.0;
        }   // end for
    }   // end for

    Eigen::MatrixXd outMat( m+4, m+4);
    outMat << K, B, Bt, Eigen::Matrix4d::Zero(4,4);

    Eigen::MatrixXd omega = outMat.inverse();
    return omega.block(0,0,m,m);    // m * m upper left
}   // end setOmega


// public
double ObjModelPatchBendingEnergy::operator()( const IntSet& p, const IntSet& q) const
{
    Eigen::VectorXd qx, qy, qz;
    const int m = setCoordinateVectors( _m1, q, qx, qy, qz);
    assert( m == (int)p.size());
    if ( m != (int)p.size())
        return -1;

    Eigen::VectorXd px, py, pz;
    setCoordinateVectors( _m0, p, px, py, pz);
    Eigen::MatrixXd omega = setOmega( px, py, pz);
    return double(qx.transpose()*omega*qx) + double(qy.transpose()*omega*qy) + double(qz.transpose()*omega*qz);
}   // end operator()


cv::Mat_<float> setOmega( const cv::Mat_<cv::Vec3f>& p)
{
    const int m = p.rows;
    const cv::Mat_<float> fp = p.reshape(1);

    const cv::Mat_<float> xp = fp.col(0);
    const cv::Mat_<float> yp = fp.col(1);
    const cv::Mat_<float> zp = fp.col(2);
    cv::Mat_<float> K( m, m);
    for ( int i = 0; i < m; ++i)
    {
        float* krow = K.ptr<float>(i);
        for ( int j = 0; j < m; ++j)
        {
            const double sqDiff = pow(xp[i]-xp[j],2) + pow(yp[i]-yp[j],2) + pow(zp[i]-zp[j],2);
            krow[j] = (float)(sqDiff > 0.0 ? sqDiff * log( sqDiff) : 0.0);
        }   // end for
    }   // end for

    cv::Mat_<float> B( m, 4);
    B( cv::Range::all(), cv::Range(0,1)) = cv::Mat::ones( m, 1, CV_32FC1);
    B( cv::Range::all(), cv::Range(1,4)) = fp;

    cv::Mat_<float> M( m+4, m+4);
    M( cv::Range(0,m),   cv::Range(0,m)) = K;
    M( cv::Range(0,m),   cv::Range(m,m+4)) = B;
    M( cv::Range(m,m+4), cv::Range(0,m)) = B.t();
    M( cv::Range(m,m+4), cv::Range(m,m+4)) = cv::Mat::zeros(4,4,CV_32FC1);

    return M.inv()( cv::Range(0,m), cv::Range(0,m));
}   // end setOmega


// public static
double ObjModelPatchBendingEnergy::calc( const cv::Mat_<cv::Vec3f>& p, const cv::Mat_<cv::Vec3f>& q)
{
    assert( p.rows == q.rows);
    if ( p.rows != q.rows)
        return -1;

    const cv::Mat_<float> omega = setOmega( p);

    const cv::Mat_<float> fq = q.reshape(1);
    const cv::Mat_<float> x = fq.col(0);
    const cv::Mat_<float> y = fq.col(1);
    const cv::Mat_<float> z = fq.col(2);

    // Needs ctor cast to resolve calculation ambiguity.
    return cv::Mat_<float>( (x.t() * omega * x) + (y.t() * omega * y) + (z.t() * omega * y)).at<float>(0,0);
}   // end calc


// public static
double ObjModelPatchBendingEnergy::calcE( const cv::Mat_<cv::Vec3f>& p, const cv::Mat_<cv::Vec3f>& q)
{
    const int m = p.rows;
    assert( m == q.rows);
    if ( m != q.rows)
        return -1;

    Eigen::VectorXd qx, qy, qz;
    setCoordinateVectors( q, qx, qy, qz);

    Eigen::VectorXd px, py, pz;
    setCoordinateVectors( p, px, py, pz);
    Eigen::MatrixXd omega = setOmega( px, py, pz);
    return double(qx.transpose()*omega*qx) + double(qy.transpose()*omega*qy) + double(qz.transpose()*omega*qz);
}   // end calcE

