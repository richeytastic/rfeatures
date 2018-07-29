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

#include <ObjModelCurvatureMap.h>
#include <FeatureUtils.h>
using RFeatures::ObjModelNormals;
using RFeatures::ObjModelPolygonAreas;
using RFeatures::ObjModelCurvatureMap;
using RFeatures::ObjModel;
using RFeatures::Edge;
#include <algorithm>
#include <cassert>
#include <cfloat>
#include <cmath>


// public static
ObjModelCurvatureMap::Ptr ObjModelCurvatureMap::create( const ObjModel* m, const ObjModelNormals* n, const ObjModelPolygonAreas* a)
{
    return Ptr( new ObjModelCurvatureMap( m, n, a), [](auto x){delete x;});
}   // end create


// private
ObjModelCurvatureMap::ObjModelCurvatureMap( const ObjModel* m, const ObjModelNormals* n, const ObjModelPolygonAreas* a)
    : _model(m), _normals(n), _pareas(a)
{
}   // end ctor


void ObjModelCurvatureMap::map( const IntSet& vidxs)
{
    std::for_each( std::begin(vidxs), std::end(vidxs), [this](int v){ setWeightedVertexNormal(v);});
    std::for_each( std::begin(vidxs), std::end(vidxs), [this](int v){ setEdgeFaceSums(v);});
    std::for_each( std::begin(vidxs), std::end(vidxs), [this](int v){ setVertexAdjFaceSums(v);});
    std::for_each( std::begin(vidxs), std::end(vidxs), [this](int v){ setVertexCurvature(v);});
}   // end map


// public
const cv::Vec3d& ObjModelCurvatureMap::vertexPC1( int vi, double& kp1) const
{
    const Curvature& c = _vtxCurvature.at(vi);
    kp1 = c.kp1;
    return c.T1;
}   // end vertexPC1


// public
const cv::Vec3d& ObjModelCurvatureMap::vertexPC2( int vi, double& kp2) const
{
    const Curvature& c = _vtxCurvature.at(vi);
    kp2 = c.kp2;
    return c.T2;
}   // end vertexPC2


// public
double ObjModelCurvatureMap::vertexAdjFacesSum( int vidx) const
{
    assert( _model->getVertexIds().count(vidx) > 0);
    assert( _vtxAdjFacesSum.count(vidx) > 0);
    return _vtxAdjFacesSum.at(vidx);
}   // end vertexAdjFacesSum


// public
const cv::Vec3d& ObjModelCurvatureMap::weightedVertexNormal( int vidx) const
{
    assert( _model->getVertexIds().count(vidx) > 0);
    assert( _vtxNormals.count(vidx) > 0);
    return _vtxNormals.at(vidx);
}   // end weightedVertexNormal


// private
void ObjModelCurvatureMap::setWeightedVertexNormal( int vidx)
{
    cv::Vec3d nrm(0,0,0);
    for ( int fid : _model->getFaceIds(vidx))
    {
        assert( _normals->isPresent(fid));
        assert( _pareas->isPresent(fid));
        double faceArea = _pareas->area(fid);
        const cv::Vec3d& nrmVec = _normals->normal(fid);
        nrm[0] += faceArea * nrmVec[0]; // Weight by area of poly
        nrm[1] += faceArea * nrmVec[1]; // Weight by area of poly
        nrm[2] += faceArea * nrmVec[2]; // Weight by area of poly
    }   // end foreach
    cv::normalize( nrm, _vtxNormals[vidx]);
}   // end setWeightedVertexNormal


// private
void ObjModelCurvatureMap::setEdgeFaceSums( int vidx)
{
    // Set each edge initial value as the sum of the areas of the adjacent faces.
    // Each edge should only be adjacent to either a single face if on the boundary,
    // or two faces otherwise.
    const IntSet& edgeIds = _model->getEdgeIds( vidx);
    for ( int eid : edgeIds)
    {
        if ( _edgeFaceSums.count(eid) > 0)  // Don't recalculate if already present
            continue;

        const Edge& edge = _model->getEdge( eid);
        const IntSet& sfids = _model->getSharedFaces( eid);
        assert( sfids.size() == 1 || sfids.size() == 2); // Only valid if size of sharedFaceIds is 1 or 2!
        double esum = 0;
        std::for_each( std::begin(sfids), std::end(sfids), [&](int fid){ esum += _pareas->area(fid);});
        _edgeFaceSums[eid] = esum;   // Needs normalising
    }   // end foreach
    _vtxEdgeIds[vidx] = edgeIds;    // Copy in
}   // end setEdgeFaceSums


// private
void ObjModelCurvatureMap::setVertexAdjFaceSums( int vidx)
{
    double fsum = 0;
    const IntSet& fids = _model->getFaceIds( vidx);
    std::for_each( std::begin(fids), std::end(fids), [&](int fid){ fsum += _pareas->area(fid);});
    _vtxAdjFacesSum[vidx] = fsum;
}   // end setVertexAdjFaceSums


// public static
double ObjModelCurvatureMap::calcGivensRotation( double a, double b, double &c, double &s)
{
    double r;
    //if ( fabs(b) < DBL_MIN)
    if ( b == 0.0)
    {
        c = copysign(1.0, a);
        s = 0.0;
        r = fabs(a);
    }   // end if
    //else if ( fabs(a) < DBL_MIN)
    else if ( a == 0.0)
    {
        c = 0.0;
        s = -copysign(1.0, b);
        r = fabs(b);
    }   // end else if
    else if ( fabs(a) > fabs(b))
    {
        double t = b/a;
        double u = copysign(1.0, a) * fabs(sqrt(1.0 + t*t));
        c = 1.0/u;
        s = -c * t;
        r = a * u;
    }   // end else if
    else
    {
        double t = a/b;
        double u = copysign(1.0, b) * fabs(sqrt(1.0 + t*t));
        s = -1.0/u;
        c = -s * t;
        r = b * u;
    }   // end else
    return r;
}   // end calcGivensRotation


// private
void ObjModelCurvatureMap::setVertexCurvature( int vi)
{
    const IntSet& edgeIds = _model->getEdgeIds( vi);

    cv::Matx33d M( 0, 0, 0,
                   0, 0, 0,
                   0, 0, 0);
    std::for_each( std::begin(edgeIds), std::end(edgeIds), [&](int eid){ addEdgeCurvature( vi, eid, M);});

    const cv::Vec3d& N = weightedVertexNormal(vi);
    const cv::Vec3d E1(1,0,0);
    cv::Vec3d W;
    if ( cv::norm(E1 - N) > cv::norm(E1 + N))
        cv::normalize( E1 - N, W);
    else
        cv::normalize( E1 + N, W);

    static const cv::Matx33d I( 1, 0, 0,
                                0, 1, 0,
                                0, 0, 1);
    const cv::Matx33d Q = I - 2 * W * W.t();  // Householder matrix
    // First column of Q is N or -N depending on above +/- conditional.
    // The second and thirds columns of Q define an orthonormal basis of the tangent space
    // (but not necessarily the principal directions).
    const cv::Matx31d Ti = Q.col(1);
    const cv::Matx31d Tj = Q.col(2);

    const cv::Matx33d H = Q.t() * M * Q;
    const double a = H(1,1);  // Eigenvalue of principal curvature
    const double b = H(2,2);  // Eigenvalue of principal curvature
    double c, s;
    calcGivensRotation( a, b, c, s);
    const cv::Matx31d T1 = c*Ti - s*Tj;
    const cv::Matx31d T2 = s*Ti + c*Tj;

    Curvature& curvature = _vtxCurvature[vi];
    curvature.T1 = cv::Vec3d( T1(0), T1(1), T1(2));
    curvature.T2 = cv::Vec3d( T2(0), T2(1), T2(2));
    curvature.kp1 = 3*a - b;
    curvature.kp2 = 3*b - a;
}   // end setVertexCurvature


// private
void ObjModelCurvatureMap::addEdgeCurvature( int vi, int eid, cv::Matx33d& M)
{
    const Edge& edge = _model->getEdge( eid);
    // Get the other vertex that isn't vidx
    int vj = edge.v0;
    if ( vj == vi)
        vj = edge.v1;

    const cv::Vec3d ui = (cv::Vec3d)_model->vtx(vi);
    const cv::Vec3d uj = (cv::Vec3d)_model->vtx(vj);
    const cv::Vec3d& N = weightedVertexNormal(vi);   // Normal to the surface at vi

    static const cv::Matx33d I( 1, 0, 0,
                                0, 1, 0,
                                0, 0, 1);

    // Calc Tij as the unit vector in the tangent plane to the surface at point vi
    const cv::Matx33d nmat = I - N*N.t();
    cv::Vec3d Tij;
    cv::normalize( nmat * (ui - uj), Tij);

    const double k = 2.0 * N.dot( uj - ui) / cv::norm( uj - ui); // Approximate the directional curvature

    // Calc edge weight ensuring that sum of the edge weights for all vertices connected to vi == 1
    const double w = _edgeFaceSums.at(eid) / _vtxAdjFacesSum.at(vi);

    M += w * k * Tij * Tij.t(); // Add the weighted curvature matrix
}   // end addEdgeCurvature
