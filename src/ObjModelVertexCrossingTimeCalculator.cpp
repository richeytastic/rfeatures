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

#include <ObjModelVertexCrossingTimeCalculator.h>
#include <ObjModelFaceUnfoldingVertexSearcher.h>
#include <algorithm>    // std::swap
#include <cassert>
#include <iostream>
#include <boost/foreach.hpp>
using RFeatures::ObjModelVertexCrossingTimeCalculator;
using RFeatures::ObjModelFaceAngleCalculator;
using RFeatures::ObjModelFaceUnfoldingVertexSearcher;
using RFeatures::ObjModel;


struct ObjModelVertexCrossingTimeCalculator::VCrossingTimes
{
    explicit VCrossingTimes( boost::unordered_map<int, double> &tm)
        : _srcID(-1), _stimes(&tm), _mtimes(NULL)
    {}   // end ctor


    VCrossingTimes( boost::unordered_map<int, boost::unordered_map<int, double> > &tm, int srcID)
        : _srcID(srcID), _stimes(NULL), _mtimes(&tm)
    {
        assert( srcID >= 0);
    }   // end ctor


    double at( int i)
    {
        if ( _stimes)
        {
            if ( !_stimes->count(i))
                (*_stimes)[i] = DBL_MAX;
            return _stimes->at(i);
        }   // end if

        if ( !_mtimes->count(i) || !_mtimes->at(i).count(_srcID))
            (*_mtimes)[i][_srcID] = DBL_MAX;
        return _mtimes->at(i).at(_srcID);
    }   // end at()

private:
    const int _srcID;   // For _mtimes only
    boost::unordered_map<int, double> *_stimes; // Times from a single source per vertex
    boost::unordered_map<int, boost::unordered_map<int, double> > *_mtimes; // Times from multiple sources per vertex
};  // end struct



// public
ObjModelVertexCrossingTimeCalculator::ObjModelVertexCrossingTimeCalculator( const ObjModel::Ptr m,
                                                                            boost::unordered_map<int, double> &tm,
                                                                            RFeatures::FaceAngles *fa)
    : _model(m), _faceAngles(fa), _vtimes( new ObjModelVertexCrossingTimeCalculator::VCrossingTimes( tm))
{
}   // end ctor


// public
ObjModelVertexCrossingTimeCalculator::ObjModelVertexCrossingTimeCalculator( const ObjModel::Ptr m,
                                                                            boost::unordered_map<int, boost::unordered_map<int, double> > &tm,
                                                                            int srcID,
                                                                            RFeatures::FaceAngles *fa)
    : _model(m), _faceAngles(fa), _vtimes( new ObjModelVertexCrossingTimeCalculator::VCrossingTimes( tm, srcID))
{
}   // end ctor


// public
ObjModelVertexCrossingTimeCalculator::~ObjModelVertexCrossingTimeCalculator()
{
    delete _vtimes;
}   // end dtor



// public
double ObjModelVertexCrossingTimeCalculator::operator()( int C, double F)
{
    static const double HALF_PI = CV_PI/2;
    double tC = _vtimes->at(C);

    assert( _model->getVertexIds().count(C));
    const cv::Vec3f& vC = _model->getVertex(C);

    // Over each triangle that vertex C is a corner of, calculate the time at which the front arrives
    // based on the arrival time for the other two vertices of the triangle. If the angle is acute, the adjacent
    // triangles must be "unfolded" until a pseudo vertex can be found which gives an acute triangulation.
    const IntSet& fids = _model->getFaceIds(C);
    BOOST_FOREACH ( int fid, fids)
    {
        double theta;
        // Get cached angle on face fid at C
        if ( _faceAngles && _faceAngles->count(fid) > 0 && _faceAngles->at(fid).count(C) > 0)
            theta = _faceAngles->at(fid).at(C);
        else
        {
            theta = ObjModelFaceAngleCalculator::calcInnerAngle(_model,fid, C);
            if ( _faceAngles)
                (*_faceAngles)[fid][C] = theta;    // Cache
        }   // end else

        int A, B;
        const RFeatures::ObjPoly& face = _model->getFace( fid);
        face.getOpposite( C, A, B); // For each triangle connected to C, get the two vertex IDs on the opposite edge.

        const double tA = _vtimes->at( A);
        const double tB = _vtimes->at( B);

        const cv::Vec3f& vA = _model->getVertex(A);
        const cv::Vec3f& vB = _model->getVertex(B);
        const cv::Vec3d vBC = vB - vC;
        const cv::Vec3d vAC = vA - vC;
        double a = cv::norm( vBC);
        double b = cv::norm( vAC);

        double t2v = tC;
        if ( tA  == DBL_MAX && tB < DBL_MAX)
            t2v = F*a + tB;
        else if ( tB == DBL_MAX && tA < DBL_MAX)
            t2v = F*b + tA;
        else if ( tA < DBL_MAX && tB < DBL_MAX)
        {
            t2v = std::min<double>( F*a + tB, F*b + tA);
            if ( theta < HALF_PI)
                t2v = calcTimeAtC( tB, tA, a, b, theta, F);
            else
            {
                // "Unfold" adjacent triangles until a vertex is found that is within the planar section defined by triangle
                // fid that allows for a segmentation of triangle fid into two acute triangles. Then calculate the time to C
                // using these two psuedo triangles and use the lesser of the two times.
                cv::Vec3f vP;
                int P = ObjModelFaceUnfoldingVertexSearcher( _model)( C, fid, theta, vP);
                if ( P >= 0)
                {
                    const cv::Vec3d vPC = vP - vC;
                    const double p = cv::norm( vP - vC);
                    const double tP = _vtimes->at(P);
                    const double thetaBP = acos( vPC.dot(vBC) / (p*a));
                    const double thetaAP = acos( vPC.dot(vAC) / (p*b));
                    const double t2v0 = calcTimeAtC( tB, tP, a, p, thetaBP, F);
                    const double t2v1 = calcTimeAtC( tA, tP, b, p, thetaAP, F);
                    t2v = std::min<double>( t2v0, t2v1);
                }   // end if
            }   // end else
        }   // end if

        tC = std::min<double>( t2v, tC);
    }   // end foreach

    return tC;
}   // end operator()


// public static
// Calculate the time to vertex C. See breakdown on page 8433 of "Computing Geodesic Paths on Manifolds".
double ObjModelVertexCrossingTimeCalculator::calcTimeAtC( double tB, double tA, double a, double b, double thetaAtC, double F)
{
    if ( tB < tA)
    {
        std::swap( tB, tA);
        std::swap( a, b);
    }   // end if

    // Comments are example values produced
    const double u = tB - tA;   // 0
    assert( u >= 0.0);
    const double cosTheta = cos(thetaAtC);  // 0

    const double aSq = pow(a,2);    // 1
    const double bSq = pow(b,2);    // 1

    // Get the 3 quadratic terms
    const double qA = aSq + bSq - 2*a*b*cosTheta;   // 2
    const double qB = 2*b*u*(a*cosTheta - b);       // 0
    const double qC = bSq*(pow(u,2) - pow(F,2)*aSq*pow(sin(thetaAtC),2));   // -1

    // Quadratic calc
    const double sRT = sqrt( std::max<double>(0.0, pow(qB,2) - 4*qA*qC));   // 2*sqrt(2) = 2.828427
    const double t = (sRT - qB)/(2*qA); // sqrt(2)/2

    double CD = b*(t-u);
    if ( t != 0.0)
        CD /= t;

    double tC;
    if ( u < t && a*cosTheta < CD && CD*cosTheta < a)
        tC = tA + t;
    else
        tC = std::min<double>( F*b + tA, F*a + tB);

    return tC;
}   // end calcTimeAtC
