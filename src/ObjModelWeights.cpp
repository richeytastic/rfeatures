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

#include <ObjModelTools.h>
#include <cassert>
using RFeatures::InlierWeightCalculator;
using RFeatures::ObjModel;


InlierWeightCalculator::InlierWeightCalculator( const ObjModel& m, double k, double C)
    : _model(m), _C( -fabs(C)), _lambda( exp( _C * pow(fabs(k),2)))
{
    assert(m.hasSequentialVertexIds());
    // Initialise vertex weights and displacements.
    const int NV = m.numVtxs();
    for ( int i = 0; i < NV; ++i)
    {
        _vw.at<float>(i) = 1;
        _vsd.at<float>(i) = -1;
    }   // end for
}   // end ctor


const cv::Mat_<float>& InlierWeightCalculator::updateWeights( const ObjModel& pmodel, const IntSet* mask)
{
    assert( pmodel.hasSequentialVertexIds());
    double dsumsq = 0;    // Sum of weighted squares of displacements over vertices
    double wsum = 0;

    const int NP = pmodel.numVtxs();
    const int NV = _model.numVtxs();
    for ( int vidx = 0; vidx < NV; ++vidx)
    {
        _vsd.at<float>(vidx) = -1;
        double w = _vw.at<float>(vidx);
        wsum += w;

        if ( vidx < NP && ( !mask || mask->count(vidx) == 0))
        {
            const cv::Vec3f& tv = _model.vtx(vidx);        // Target vertex position
            const cv::Vec3f& pv = pmodel.vtx(vidx);        // Argument vertex position
            const double sd = l2sq( tv - pv);  // Squared displacement
            _vsd.at<float>(vidx) = float(sd);
            dsumsq += w * sd;
        }   // end if
    }   // end for

    const double ssigma = dsumsq / pow(wsum,2);    // Squared sigma

    for ( int vidx = 0; vidx < NV; ++vidx)
    {
        if ( _vsd.at<float>(vidx) < 0)
            _vw.at<float>(vidx) = 0;
        else
        {
            const double sz = _vsd.at<float>(vidx) / ssigma;  // Squared z-score
            const double ip = exp( _C * sz);
            _vw.at<float>(vidx) = float(ip / (ip + _lambda));    // Update the weight for this vertex
        }   // end else
    }   // end for

    return _vw;
}   // end updateWeights
