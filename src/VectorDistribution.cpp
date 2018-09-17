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

#include <VectorDistribution.h>
using RFeatures::VectorDistribution;
#include <cassert>


VectorDistribution::VectorDistribution( int vlen, int nbins, float minVal, float maxVal)
    : _vlen(vlen), _nbins(nbins), _minVal(minVal), _maxVal(maxVal), _count(0)
{
    assert( vlen >= 1);
    assert( nbins >= 2);
    assert( maxVal > minVal);

    for ( int i = 0; i < vlen; ++i)
    {
        _dist.push_back(vector<double>(nbins));
        _vbins.push_back(vector<double>(nbins));
        for ( int j = 0; j < nbins; ++j)
        {
            _vbins[i][j] = 1;   // Laplace smoothing - prevent extreme likelihoods
            _dist[i][j] = 1./nbins;
        }   // end for
    }   // end for
}   // end ctor


cv::Mat_<float> formatCvVec( const cv::Mat_<float>& vec, int vlen)
{
    // Ensure v is a row vector
    cv::Mat_<float> v = vec;
    if ( vec.rows > 1)
        v = vec.t();
    if ( v.total() != (size_t)vlen)
        throw VectorLengthException( "VectorDistribution::addVector: invalid vector length!");
    return v;
}   // end formatCvVec


void VectorDistribution::addVector( const cv::Mat_<float>& vec) throw (VectorLengthException)
{
    const cv::Mat_<float> v = formatCvVec( vec, _vlen);
    addVector( v.ptr<float>(0));
}   // end addVector


void VectorDistribution::addVector( const vector<float>& vec) throw (VectorLengthException)
{
    if ( vec.size() != (size_t)_vlen)
        throw VectorLengthException( "VectorDistribution::addVector: invalid vector length!");
    addVector( &vec[0]);
}   // end addVector


void VectorDistribution::getLikelihood( const cv::Mat_<float>& vec, float* lhoods) throw (VectorLengthException)
{
    const cv::Mat_<float> v = formatCvVec( vec, _vlen);
    getLikelihood( v.ptr<float>(0), lhoods);
}   // end getLikelihood


void VectorDistribution::getLikelihood( const vector<float>& vec, float* lhoods) throw (VectorLengthException)
{
    if ( vec.size() != (size_t)_vlen)
        throw VectorLengthException( "VectorDistribution::addVector: invalid vector length!");
    getLikelihood( &vec[0], lhoods);
}   // end getLikelihood


#include <algorithm>
#include <stdexcept>
#include <sstream>

// private
void VectorDistribution::getLikelihood( const float* vec, float* lhoods)
{
    const double minVal = _minVal;
    const double maxVal = _maxVal;
    const double rng = maxVal - minVal;
    const int vlen = _vlen;
    const int nbins = _nbins;
    for ( int i = 0; i < vlen; ++i)
    {
        const double v = vec[i];
        if ( v >= maxVal || v < minVal)
        {
            std::ostringstream oss;
            oss << "VectorDistribution::addVector: " << v << " not in allowed interval [" << minVal << ", " << maxVal << ")";
            throw std::out_of_range( oss.str());
        }   // end if

        const int b = std::min<int>( int(v/rng * nbins), nbins-1);
        lhoods[i] = float(_dist[i][b]);
    }   // end for
}   // end getLikelihood


// private
void VectorDistribution::addVector( const float* vec)
{
    const double minVal = _minVal;
    const double maxVal = _maxVal;
    const double rng = maxVal - minVal;
    const int vlen = _vlen;
    const int nbins = _nbins;
    for ( int i = 0; i < vlen; ++i)
    {
        const double v = vec[i];
        if ( v >= maxVal || v < minVal)
        {
            std::ostringstream oss;
            oss << "VectorDistribution::addVector: " << v << " not in allowed interval [" << minVal << ", " << maxVal << ")";
            throw std::out_of_range( oss.str());
        }   // end if

        const int b = std::min<int>( int(v/rng * nbins), nbins-1);
        _vbins[i][b] += 1;
    }   // end for

    _count++;
    updateDistribution();
}   // end addVector


// private
void VectorDistribution::updateDistribution()
{
    // Normalise the distribution for each bin
    const int denom = _nbins + _count;
    for ( int i = 0; i < _vlen; ++i)
        for ( int j = 0; j < _nbins; ++j)
            _dist[i][j] = _vbins[i][j]/denom;
}   // end updateDistribution

