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

#include "ObjPolyInterpolator.h"
#include "ObjModelVertexCrossingTimeCalculator.h"
using RFeatures::ObjPolyInterpolator;
using RFeatures::ObjModel;
using RFeatures::ObjPoly;
using RFeatures::ObjModelFastMarcher;
#include "FeatureUtils.h"
#include <boost/foreach.hpp>
#include <algorithm>
#include <iomanip>
#include <cassert>


// public
ObjPolyInterpolator::ObjPolyInterpolator( const ObjModel::Ptr imod,
                                          const ObjModel::Ptr omod,
                                          const boost::unordered_map<int, int>& nsources,
                                          const boost::unordered_map<int, boost::unordered_map<int, double> >& itimes)
                                
    : _inmod(imod), _outmod(omod), _nearestSources(nsources), _itimes(itimes)
{
}   // end ctor


// public static
double ObjPolyInterpolator::calcLambda( double txa, double txb, double tya, double tyb)
{
    assert( tya >= txa);
    double lambda = 0.5;
    const double divisor = txb - txa + tya - tyb;
    if ( divisor != 0.0)
        lambda = (tya - txa)/divisor;
    return lambda;
}   // end calcLambda


// public static
cv::Vec3d ObjPolyInterpolator::calcLambdaPoint( double lambda, const cv::Vec3d& vA, const cv::Vec3d& vB)
{
    return (1.0 - lambda)*vA + lambda*vB;
}   // end calcLambdaPoint


// private
void ObjPolyInterpolator::printDebug( int A, int a) const
{
    if ( A != 1332)
        return;

    const int x = _nearestSources.at(a);
    const cv::Vec3d vA = _inmod->vtx(a);
    const cv::Vec3d vX = _outmod->vtx(x);

    if ( A == a)
        std::cerr << " #";
    else
        std::cerr << "  ";
    std::cerr << " (" << a << ") near [" << x << "]" << std::endl;
    std::cerr << "     (" << std::fixed << std::setprecision(2) << std::setw(7)
              << vA[0] << ", " << vA[1] << ")"
              << " near [" << vX[0] << ", " << vX[1] << "]" << std::endl;
}   // end printDebug


// private
double ObjPolyInterpolator::calcTriangleArea( int X, int Y, int Z) const
{
    const cv::Vec3f& vX = _outmod->vtx(X);
    const cv::Vec3f& vY = _outmod->vtx(Y);
    const cv::Vec3f& vZ = _outmod->vtx(Z);
    return RFeatures::calcTriangleArea( vX, vY, vZ);
}   // end calcTriangleArea


// public
cv::Vec3f ObjPolyInterpolator::interpolate( int A) const
{
    const int X = _nearestSources.at(A);  // -1 before any sources
    if ( X < 0)
        return _inmod->getVertex(A);

    const cv::Vec3d vA = _inmod->getVertex(A);
    const double txa = _itimes.at(A).at(X);

    int icount = 4; // Found empirically to result in more equalateral triangles than 5,3,2, or 1.
    cv::Vec3d mpos = icount*vA;

    const IntSet& cuvs = _inmod->getConnectedVertices( A);
    BOOST_FOREACH ( const int& B, cuvs)
    {
        const int Y = _nearestSources.at(B);
        if ( Y == X)
            continue;

        const cv::Vec3d vB = _inmod->getVertex(B);
        const double tya = _itimes.at(A).at(Y);
        const double txb = _itimes.at(B).at(X);
        const double tyb = _itimes.at(B).at(Y);
        const double l0 = calcLambda( txa, txb, tya, tyb);  // lambda position between A and B
        mpos += calcLambdaPoint( l0, vA, vB);
        icount++;
    }   // end foreach

    const double SCALEF = 1./icount;
    return cv::Vec3f( float(mpos[0] * SCALEF), float(mpos[1] * SCALEF), float(mpos[2] * SCALEF));
}   // end interpolate
