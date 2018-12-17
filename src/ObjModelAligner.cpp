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
using RFeatures::ObjModelICPAligner;
using RFeatures::ObjModelProcrustesSuperimposition;
using RFeatures::VertexWeights;
using RFeatures::ObjModel;
#include <icpPointToPlane.h>    // Andreas Geiger
#include <iostream>
#include <cstring>
#include <cassert>
#include <vector>
#include <cmath>


ObjModelProcrustesSuperimposition::ObjModelProcrustesSuperimposition( const ObjModel* model, const VertexWeights* vw, bool scaleUp)
    : _scaleUp(scaleUp)
{
    const int n = int(model->numVertices());

    _A = verticesToCvMat( model);   // 3 rows x n columns
    assert( _A.size().width == n);

    bool makeWeights = !vw;
    if ( vw && int(vw->size()) != n)
    {
        std::cerr << "[WARNING] RFeatures::ObjModelProcrustesSuperimposition::ctor: "
                  << "IGNORING WEIGHTS since size does not match number of vertices in model!" << std::endl;
        makeWeights = true;
    }   // end if

    if (makeWeights)
        _W = cv::Mat::ones( 1, n, CV_64FC1);
    else
        _W = weightsToCvMat( *vw);

    _vbar = calcMeanColumnVector( _A, _W);
    _s = toMean( _A, _vbar, _W);
    scale( _A, 1.0/_s);
}   // end ctor


cv::Matx44d ObjModelProcrustesSuperimposition::calcTransform( const ObjModel* model) const
{
    const int n = _A.size().width;
    if ( static_cast<int>(model->getNumVertices()) != n)
    {
        std::cerr << "[WARNING] RFeatures::ObjModelProcrustesSuperimposition::calcTransform: "
                  << "Column vector count mismatch between argument model and target model!" << std::endl;
        return cv::Matx44d(1,0,0,0,
                           0,1,0,0,
                           0,0,1,0,
                           0,0,0,1); 
    }   // end if

    cv::Mat_<double> B = verticesToCvMat( model);
    cv::Vec3d vbar = calcMeanColumnVector( B, _W);
    double sB = toMean( B, vbar, _W);
    scale( B, 1.0/sB);

    assert( _W.size().width == n);

    // Compute the covariance matrix between B and A. Weight B first.
    for ( int i = 0; i < n; ++i)
    {
        cv::Mat v = B.col(i);
        double w = _W.at<double>(i);
        v.at<double>(0) *= w;
        v.at<double>(1) *= w;
        v.at<double>(2) *= w;
    }   // end for
    cv::Mat C = B * _A.t();

    cv::Mat s, U, Vt;
    cv::SVD::compute( C, s, U, Vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

    const double scaleFactor = _scaleUp ? _s/sB : 1;
    cv::Mat R = (Vt.t() * U.t()) * scaleFactor;   // 3x3 rotation matrix with scaling

    cv::Matx44d t0( R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), _vbar[0],
                    R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), _vbar[1],
                    R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), _vbar[2],
                                    0,                0,                 0,        1);

    cv::Matx44d t1( 1, 0, 0, -vbar[0],
                    0, 1, 0, -vbar[1],
                    0, 0, 1, -vbar[2],
                    0, 0, 0,       1);

    return t0 * t1;
}   // end calcTransform



namespace {

void setVertex( double* mpoints, const cv::Vec3f& v)
{
    mpoints[0] = v[0];
    mpoints[1] = v[1];
    mpoints[2] = v[2];
}   // end setVertex


double* createModelPointsArray( const ObjModel* model, int& N)
{
    N = (int)model->getNumVertices();
    assert( N >= 5);

    double* mpoints = new double[3*N];
    int k = 0;
    const IntSet& vidxs = model->getVertexIds();
    for ( int vidx : vidxs)
    {
        setVertex( &mpoints[k*3], model->vtx(vidx));
        k++;
    }   // end foreach
    return mpoints;
}   // end createModelPointsArray

}   // end namespace


ObjModelICPAligner::ObjModelICPAligner( const ObjModel* m) { _T = createModelPointsArray( m, _n);}
ObjModelICPAligner::~ObjModelICPAligner() { delete[] _T;}


cv::Matx44d ObjModelICPAligner::calcTransform( const ObjModel* model) const
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

