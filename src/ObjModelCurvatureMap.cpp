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
#include <ObjModelNormalCalculator.h>
#include <ObjModelPolygonAreaCalculator.h>
#include <ObjModelMeshTraversalRecorder.h>
#include <FeatureUtils.h>
using RFeatures::ObjModelNormalCalculator;
using RFeatures::ObjModelPolygonAreaCalculator;
using RFeatures::ObjModelCurvatureMap;
using RFeatures::ObjModel;
using RFeatures::Edge;
#include <boost/foreach.hpp>
#include <cmath>
#include <cfloat>

class ObjModelCurvatureMap::Deleter
{ public:
    void operator()( ObjModelCurvatureMap* p) { delete p;}
};  // end class


// public static
ObjModelCurvatureMap::Ptr ObjModelCurvatureMap::create( ObjModel::Ptr model, int sfid)
{
    return Ptr( new ObjModelCurvatureMap( model, sfid), Deleter());
}   // end create


// private
ObjModelCurvatureMap::ObjModelCurvatureMap( ObjModel::Ptr model, int sfid)
    : _model( model),
    _faceNorms( new ObjModelNormalCalculator( _model)),
    _faceAreas( new ObjModelPolygonAreaCalculator( _model))
{
    RFeatures::ObjModelTriangleMeshParser parser( _model);
    parser.addTriangleParser( _faceNorms);
    parser.addTriangleParser( _faceAreas);
    RFeatures::ObjModelMeshTraversalRecorder vrecorder;
    parser.addTriangleParser( &vrecorder);
    parser.parse( sfid, cv::Vec3d(0,0,1));

    const IntSet& vidxs = vrecorder.getTraversedVertices();

    const IntSet& fids = parser.getParsedFaces();
    bool failParse = false;
    if ( fids.size() != model->getFaceIds().size())
    {
        std::cerr << "[WARNING] RFeatures::ObjModelCurvatureMap: there are more polygons in the model than were parsed!" << std::endl;
        failParse = true;
    }   // end if
    if ( vidxs.size() != model->getVertexIds().size())
    {
        std::cerr << "[WARNING] RFeatures::ObjModelCurvatureMap: there are more vertices in the model than were parsed!" << std::endl;
        failParse = true;
    }   // end if
    if ( failParse)
        std::cerr << "\t This error is usually because the model has at least two different components disconnected from one another." << std::endl;

    BOOST_FOREACH ( int vidx, vidxs)
        calcVertexNormal( vidx);

    BOOST_FOREACH ( int vidx, vidxs)
        calcEdgeFaceSums( vidx);

    BOOST_FOREACH ( int vidx, vidxs)
        calcVertexAdjFaceSums( vidx);

    BOOST_FOREACH ( int vidx, vidxs)
        calcVertexCurvature( vidx);
}   // end ctor


// private
ObjModelCurvatureMap::~ObjModelCurvatureMap()
{
    delete _faceAreas;
    delete _faceNorms;
}   // end dtor


// public
void ObjModelCurvatureMap::recalcVertex( int vidx)
{
    // Recalculate the polygon areas normals:
    const IntSet& sfids = _model->getFaceIds( vidx);
    BOOST_FOREACH ( int fid, sfids)
    {
        _faceAreas->recalcPolygonArea( fid);
        _faceNorms->recalcFaceNormal( fid);
    }   // end foreach

    calcVertexNormal( vidx);
    calcEdgeFaceSums( vidx);
    calcVertexAdjFaceSums( vidx);
    calcVertexCurvature( vidx);

    const IntSet& cvs = _model->getConnectedVertices( vidx);
    BOOST_FOREACH ( int cv,cvs)
    {
        calcVertexNormal( cv);
        calcEdgeFaceSums( cv);
        calcVertexAdjFaceSums( cv);
        calcVertexCurvature( cv);
    }   // end foreach
}   // end recalcVertex


// public
void ObjModelCurvatureMap::recalcFace( int fid)
{
    _faceAreas->recalcPolygonArea( fid);
    _faceNorms->recalcFaceNormal( fid);
    const int* vtxs = _model->getFaceVertices(fid);
    for ( int i = 0; i < 3; ++i)
    {
        calcVertexNormal( vtxs[i]);
        calcEdgeFaceSums( vtxs[i]);
        calcVertexAdjFaceSums( vtxs[i]);
        calcVertexCurvature( vtxs[i]);
    }   // end for
}   // end recalcFace


// public
const cv::Vec3d& ObjModelCurvatureMap::getVertexPrincipalCurvature1( int vi, double& kp1) const
{
    const Curvature& c = _vtxCurvature.at(vi);
    kp1 = c.kp1;
    return c.T1;
}   // end getVertexPrincipalCurvature1


// public
const cv::Vec3d& ObjModelCurvatureMap::getVertexPrincipalCurvature2( int vi, double& kp2) const
{
    const Curvature& c = _vtxCurvature.at(vi);
    kp2 = c.kp2;
    return c.T2;
}   // end getVertexPrincipalCurvature2


// public
double ObjModelCurvatureMap::getFaceArea( int fid) const
{
    double area = 0.0;
    if ( _faceAreas->isPresent(fid))
        area = _faceAreas->getPolygonArea( fid);
    return area;
}   // end getFaceArea


// public
double ObjModelCurvatureMap::getVertexAdjFacesSum( int vi) const
{
    double adjFacesSum = 0.0;
    if ( _vtxAdjFacesSum.count(vi) > 0)
        adjFacesSum = _vtxAdjFacesSum.at(vi);
    return adjFacesSum;
}   // end getVertexAdjFacesSum


// public
const cv::Vec3d& ObjModelCurvatureMap::getVertexNormal( int vidx) const
{
    static const cv::Vec3d NULL_VECTOR(0,0,0);
    assert( _model->getVertexIds().count(vidx) > 0);
    const cv::Vec3d* nrm = &NULL_VECTOR;
    if ( _vtxNormals.count(vidx) > 0)
        nrm = &_vtxNormals.at(vidx);
    else
        std::cerr << "[WARNING] ObjModelCurvatureMap::getVertexNormal(" << vidx << "): NULL vector returned!" << std::endl;
    return *nrm;
}   // end getVertexNormal


// public
const cv::Vec3d& ObjModelCurvatureMap::getFaceNormal( int fid) const
{
    static const cv::Vec3d NULL_VECTOR(0,0,0);
    assert( _model->getFaceIds().count(fid) > 0);
    const cv::Vec3d* nrm = &NULL_VECTOR;
    if ( _faceNorms->isPresent(fid))
        nrm = &_faceNorms->getFaceNormal(fid);
    else
        std::cerr << "[WARNING] ObjModelCurvatureMap::getFaceNormal(" << fid << "): NULL vector returned!" << std::endl;
    return *nrm;
}   // end getFaceNormal


// private
void ObjModelCurvatureMap::calcVertexNormal( int vidx)
{
    cv::Vec3d nrm(0,0,0);
    const IntSet& fids = _model->getFaceIds(vidx);
    double faceArea;
    BOOST_FOREACH ( int fid, fids)
    {
        faceArea = getFaceArea(fid);
        const cv::Vec3d& nrmVec = getFaceNormal( fid);
        nrm[0] += faceArea * nrmVec[0]; // Weight by area of poly
        nrm[1] += faceArea * nrmVec[1]; // Weight by area of poly
        nrm[2] += faceArea * nrmVec[2]; // Weight by area of poly
    }   // end foreach
    cv::normalize( nrm, _vtxNormals[vidx]);
}   // end calcVertexNormal


// private
void ObjModelCurvatureMap::calcEdgeFaceSums( int vidx)
{
    // Set each edge initial value as the sum of the areas of the adjacent faces.
    // Each edge should only be adjacent to either a single face if on the boundary,
    // or two faces otherwise.
    const IntSet& edgeIds = _model->getEdgeIds( vidx);
    BOOST_FOREACH ( int eid, edgeIds)
    {
        if ( _edgeFaceSums.count(eid))
            continue;

        const Edge& edge = _model->getEdge( eid);
        const IntSet& sharedFaceIds = _model->getSharedFaces( edge.v0, edge.v1);
        // Only valid if size of sharedFaceIds is 1 or 2!
        assert( sharedFaceIds.size() == 1 || sharedFaceIds.size() == 2);
        if ( sharedFaceIds.size() > 2)
            std::cerr << "[ERROR] RFeatures::ObjModelCurvatureMap::calcEdgeFaceSums: Non-manifold edge encountered!" << std::endl;
        double esum = 0.0;
        BOOST_FOREACH ( int fid, sharedFaceIds)
            esum += getFaceArea(fid);
        _edgeFaceSums[eid] = esum;   // Needs normalising
    }   // end foreach
    _vtxEdgeIds[vidx] = edgeIds;    // Copy in
}   // end calcEdgeFaceSums



// private
void ObjModelCurvatureMap::calcVertexAdjFaceSums( int vidx)
{
    const IntSet& fids = _model->getFaceIds( vidx);
    double fsum = 0;
    BOOST_FOREACH ( int fid, fids)
        fsum += getFaceArea(fid);
    _vtxAdjFacesSum[vidx] = fsum;
}   // end calcVertexAdjFaceSums


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
void ObjModelCurvatureMap::calcVertexCurvature( int vi)
{
    const IntSet& edgeIds = _model->getEdgeIds( vi);

    cv::Matx33d M( 0, 0, 0,
                   0, 0, 0,
                   0, 0, 0);
    BOOST_FOREACH ( int eid, edgeIds)
        addEdgeCurvature( vi, eid, M);

    const cv::Vec3d& N = getVertexNormal(vi);
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
}   // end calcVertexCurvature



// private
void ObjModelCurvatureMap::addEdgeCurvature( int vi, int eid, cv::Matx33d& M)
{
    const Edge& edge = _model->getEdge( eid);
    // Get the other vertex that isn't vidx
    int vj = edge.v0;
    if ( vj == vi)
        vj = edge.v1;

    const cv::Vec3d ui = (cv::Vec3d)_model->getVertex(vi);
    const cv::Vec3d uj = (cv::Vec3d)_model->getVertex(vj);
    const cv::Vec3d& N = getVertexNormal(vi);   // Normal to the surface at vi

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


