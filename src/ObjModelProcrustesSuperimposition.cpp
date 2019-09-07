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

#include <ObjModelProcrustesSuperimposition.h>
#include <ObjModelTools.h>
using RFeatures::ObjModelProcrustesSuperimposition;
using RFeatures::ObjModel;


ObjModelProcrustesSuperimposition::ObjModelProcrustesSuperimposition( const ObjModel& model, const std::vector<double>& vw, bool scaleUp)
    : _model(model), _scaleUp(scaleUp)
{
    assert(model.hasSequentialVertexIds());
    const int n = model.numVtxs();

    // Warn if the number of weights does not match the number of vertices in the given model.
    if ( !vw.empty() && int(vw.size()) != n)
    {
        std::cerr << "[WARNING] RFeatures::ObjModelProcrustesSuperimposition::ctor: "
                  << "IGNORING WEIGHTS since size does not match number of vertices in model!" << std::endl;
    }   // end if

    const bool makeWeights = vw.empty() || int(vw.size()) != n;
    if (makeWeights)
        _W = cv::Mat::ones( 1, n, CV_64FC1);
    else
    {
        _W = cv::Mat_<double>( vw);  // Makes a column vector. Note that weights are shared (not copied)
        _W = _W.reshape( 0, 1);     // Make a row vector.
    }   // end else

    _A = verticesToCvMat( model);  // Model vertices as column vectors
    _vbar = calcMeanColumnVector( _A, _W);
    _s = toMean( _A, _vbar, _W);
    _A *= 1.0/_s;
}   // end ctor


cv::Matx44d ObjModelProcrustesSuperimposition::calcTransform( const ObjModel& model) const
{
    assert(model.hasSequentialVertexIds());

    cv::Matx44d t0 = cv::Matx44d::eye();
    cv::Vec3d vbar(0,0,0);

    const int n = _A.cols;
    assert( _W.cols == n);
    if ( model.numVtxs() != n)
    {
        std::cerr << "[WARNING] RFeatures::ObjModelProcrustesSuperimposition::calcTransform: "
                  << "Column vector count mismatch between argument model and target model!" << std::endl;
        assert(false);
    }   // end if
    else
    {
        cv::Mat_<double> B = verticesToCvMat( model);
        vbar = calcMeanColumnVector( B, _W);
        const double sB = toMean( B, vbar, _W);
        B *= 1.0/sB;

        // Compute the covariance between B and A. Weight B first.
        for ( int i = 0; i < n; ++i)
        {
            const double w = _W.at<double>(i);
            cv::Mat v = B.col(i);
            v.at<double>(0) *= w;
            v.at<double>(1) *= w;
            v.at<double>(2) *= w;
        }   // end for
        cv::Mat C = B * _A.t();

        cv::Mat s, U, Vt;
        cv::SVD::compute( C, s, U, Vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

        const double scaleFactor = _scaleUp ? _s/sB : 1.0;
        cv::Mat R = (Vt.t() * U.t()) * scaleFactor;   // 3x3 rotation matrix with scaling

        t0 = cv::Matx44d( R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), _vbar[0],
                          R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), _vbar[1],
                          R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), _vbar[2],
                                          0,                 0,                 0,       1);
    }   // end else

    cv::Matx44d t1( 1, 0, 0, -vbar[0],
                    0, 1, 0, -vbar[1],
                    0, 0, 1, -vbar[2],
                    0, 0, 0,       1);

    return t0 * t1;
}   // end calcTransform
