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

#pragma once
#ifndef RFEATURES_VECTOR_DISTRIBUTION_H
#define RFEATURES_VECTOR_DISTRIBUTION_H
// Disable warnings about MSVC compiler not implementing exception specifications
#ifdef _WIN32
#pragma warning( disable : 4290)
#endif

#include <vector>
using std::vector;
#include <opencv2/opencv.hpp>
#include "FeatureExceptions.h"
using RFeatures::VectorLengthException;
#include "rFeatures_Export.h"

namespace RFeatures
{

class rFeatures_EXPORT VectorDistribution
{
public:
    // minVal, maxVal define a half open interval: [minVal, maxVal)
    VectorDistribution( int vlen, int nbins, float minVal, float maxVal);

    void addVector( const cv::Mat_<float>& vec) throw (VectorLengthException);
    void addVector( const vector<float>& vec) throw (VectorLengthException);

    // Given a particular vector (vec), set array likelihoods with the independent
    // probabilities of the individual vector elements given the distribution generated
    // from the added vectors. Array likelihoods must have the same length as vec (and
    // as the generated distribution).
    void getLikelihood( const cv::Mat_<float>& vec, float* likelihoods) throw (VectorLengthException);
    void getLikelihood( const vector<float>& vec, float* likelihoods) throw (VectorLengthException);

private:
    const int _vlen;
    const int _nbins;
    const double _minVal;
    const double _maxVal;
    int _count;
    vector< vector<double> > _vbins;
    vector< vector<double> > _dist;

    void getLikelihood( const float*, float*);
    void addVector( const float*);
    // Update the probability distribution generated from the added vectors.
    void updateDistribution();
};  // end class

}   // end namespace

#endif
