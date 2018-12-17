/************************************************************************
 * Copyright (C) 2018 Richard Palmer
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
using RFeatures::InlierWeightCalculator;
using RFeatures::ObjModel;
using VW = RFeatures::VertexWeights;
using VD = RFeatures::VertexDisplacements;


InlierWeightCalculator::InlierWeightCalculator( const ObjModel* m, double k, double C)
    : _model(m), _C( -fabs(C)), _lambda( exp( _C * pow(fabs(k),2)))
{
    // Initialise vertex weights and displacements.
    for ( int vidx : m->vertexIds())
    {
        _vw[vidx] = 1;
        _vsd[vidx] = -1;
    }   // end for
}   // end ctor


const VW& InlierWeightCalculator::updateWeights( const ObjModel* pmodel, const IntSet* mask)
{
    const IntSet& tvidxs = _model->vertexIds();
    const IntSet& pvidxs = pmodel->vertexIds();

    double dsumsq = 0;    // Sum of weighted squares of displacements over vertices
    double wsum = 0;

    for ( int vidx : tvidxs)
    {
        _vsd[vidx] = -1;
        double w = _vw.at(vidx);
        wsum += w;

        if ( pvidxs.count(vidx) > 0 && ( !mask || mask->count(vidx) == 0))
        {
            const cv::Vec3f& tv = _model->vtx(vidx);        // Target vertex position
            const cv::Vec3f& pv = pmodel->vtx(vidx);        // Argument vertex position
            const double sd = _vsd[vidx] = l2sq( tv - pv);  // Squared displacement
            dsumsq += w * sd;
        }   // end if
    }   // end for

    const double ssigma = dsumsq / pow(wsum,2);    // Squared sigma

    for ( int vidx : tvidxs)
    {
        if ( _vsd.at(vidx) < 0)
            _vw[vidx] = 0;
        else
        {
            const double sz = _vsd[vidx] / ssigma;  // Squared z-score
            const double ip = exp( _C * sz);
            _vw[vidx] = ip / (ip + _lambda);    // Update the weight for this vertex
        }   // end else
    }   // end for

    return _vw;
}   // end updateWeights
