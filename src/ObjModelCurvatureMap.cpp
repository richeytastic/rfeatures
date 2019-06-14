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

#include <ObjModelCurvatureMap.h>
#include <FeatureUtils.h>
using RFeatures::ObjModelCurvatureMap;
using RFeatures::ObjModelManifolds;
using RFeatures::ObjModel;
#include <algorithm>
#include <cassert>
#include <cfloat>
#include <cmath>


// public
const cv::Vec3d& ObjModelCurvatureMap::vertexPC1( int j, int vi, double& kp1) const
{
    assert( j >= 0 && j < (int)_mdata.size());
    const Curvature& c = _mdata.at(j)->_vtxCurvature.at(vi);
    kp1 = c.kp1;
    return c.T1;
}   // end vertexPC1


// public
const cv::Vec3d& ObjModelCurvatureMap::vertexPC2( int j, int vi, double& kp2) const
{
    assert( j >= 0 && j < (int)_mdata.size());
    const Curvature& c = _mdata.at(j)->_vtxCurvature.at(vi);
    kp2 = c.kp2;
    return c.T2;
}   // end vertexPC2


// public
double ObjModelCurvatureMap::vertexAdjFacesSum( int j, int vi) const
{
    assert( j >= 0 && j < (int)_mdata.size());
    assert( _manf.cmodel()->vtxIds().count(vi) > 0);
    //assert( _mdata.at(j)->_vtxAdjFacesSum.count(vi) > 0);
    return _mdata.at(j)->_vtxAdjFacesSum.at(vi);
}   // end vertexAdjFacesSum


// public
const cv::Vec3d& ObjModelCurvatureMap::weightedVertexNormal( int j, int vi) const
{
    assert( j >= 0 && j < (int)_mdata.size());
    assert( _manf.cmodel()->vtxIds().count(vi) > 0);
    assert( _mdata.at(j)->_vtxNormals.count(vi) > 0);
    return _mdata.at(j)->_vtxNormals.at(vi);
}   // end weightedVertexNormal


// public static
ObjModelCurvatureMap::Ptr ObjModelCurvatureMap::create( const ObjModelManifolds& m)
{
    ObjModelCurvatureMap::Ptr cmap( new ObjModelCurvatureMap( m), [](ObjModelCurvatureMap* x){delete x;});
    cmap->map();
    return cmap;
}   // end create


// private
ObjModelCurvatureMap::ObjModelCurvatureMap( const ObjModelManifolds& m) : _manf(m) {}


// private
ObjModelCurvatureMap::~ObjModelCurvatureMap()
{
    for ( ManifoldData* md : _mdata)
        delete md;
}   // end dtor


// private
void ObjModelCurvatureMap::updateFace( int fid)
{
    const ObjModel* model = _manf.cmodel();
    const ObjPoly& poly = model->face(fid);
    _fareas[fid] = calcTriangleArea( model->vtx(poly[0]), model->vtx(poly[1]), model->vtx(poly[2]));
    _fnorms[fid] = model->calcFaceNorm(fid);
}   // end updateFace


// private
void ObjModelCurvatureMap::map()
{
    const IntSet& fids = _manf.cmodel()->faces();
    for ( int fid : fids)
        updateFace(fid);

    const int nm = static_cast<int>(_manf.count());
    _mdata.resize(nm);
    for ( int j = 0; j < nm; ++j)
    {
        _mdata[j] = new ManifoldData( this, _manf);
        _mdata[j]->map(j);
    }   // end for
}   // end map


void ObjModelCurvatureMap::update( int vi)
{
    IntSet mids;    // Collect all affected manifolds
    const ObjModel* model = _manf.cmodel();
    for ( int fid : model->faces(vi))
    {
        updateFace( fid);
        mids.insert( _manf.manifoldId(fid));
    }   // end for

    const IntSet& cvtxs = model->cvtxs(vi);
    for ( int j : mids)
    {
        assert( _mdata.size() > (size_t)j);
        ManifoldData* md = _mdata.at(j);

        // Collect all the affected vertices on this manifold
        std::vector<int> vidxs;
        vidxs.push_back(vi);
        for ( int cv : cvtxs)
        {
            if ( _manf.manifold(j)->vertices().count(cv) > 0)
                vidxs.push_back(cv);
        }   // end for

        std::for_each( std::begin(vidxs), std::end(vidxs), [=](int v){ md->setWeightedVertexNormal( j, v);});
        std::for_each( std::begin(vidxs), std::end(vidxs), [=](int v){ md->setEdgeFaceSums( j, v);});
        std::for_each( std::begin(vidxs), std::end(vidxs), [=](int v){ md->setVertexAdjFaceSums( j, v);});
        std::for_each( std::begin(vidxs), std::end(vidxs), [=](int v){ md->setVertexCurvature( j, v);});
    }   // end for
}   // end update


// private
ObjModelCurvatureMap::ManifoldData::ManifoldData( const ObjModelCurvatureMap* omcm, const ObjModelManifolds& manf)
    : _omcm(omcm), _manf(manf) {}


// private
void ObjModelCurvatureMap::ManifoldData::map( int j)
{
    const IntSet& vidxs = _manf.manifold(j)->vertices();
    std::for_each( std::begin(vidxs), std::end(vidxs), [=](int v){ setWeightedVertexNormal( j, v);});
    std::for_each( std::begin(vidxs), std::end(vidxs), [=](int v){ setEdgeFaceSums( j, v);});
    std::for_each( std::begin(vidxs), std::end(vidxs), [=](int v){ setVertexAdjFaceSums( j, v);});
    std::for_each( std::begin(vidxs), std::end(vidxs), [=](int v){ setVertexCurvature( j, v);});
}   // end map


// private
void ObjModelCurvatureMap::ManifoldData::setWeightedVertexNormal( int j, int vi)
{
    cv::Vec3d nrm(0,0,0);
    const IntSet& mfids = _manf.manifold(j)->polygons();
    for ( int fid : _manf.cmodel()->faces(vi))
    {
        if ( mfids.count(fid) == 0) // Face not in the manifold
            continue;

        const double farea = _omcm->faceArea(fid);
        const cv::Vec3f& nvec = _omcm->faceNorm(fid);
        nrm[0] += farea * nvec[0]; // Weight by area of poly
        nrm[1] += farea * nvec[1]; // Weight by area of poly
        nrm[2] += farea * nvec[2]; // Weight by area of poly
    }   // end for
    cv::normalize( nrm, _vtxNormals[vi]);
}   // end setWeightedVertexNormal


// private
void ObjModelCurvatureMap::ManifoldData::setEdgeFaceSums( int j, int vi)
{
    // Set each edge initial value as the sum of the areas of the adjacent faces on the manifold.
    const IntSet& mvids = _manf.manifold(j)->vertices();
    const IntSet& mfids = _manf.manifold(j)->polygons();
    assert( mvids.count(vi) > 0);
    const ObjModel* model = _manf.cmodel();
    const IntSet& cvtxs = model->cvtxs(vi);

    for ( int vj : cvtxs)
    {
        if ( mvids.count( vj) == 0) // Vertex not in the manifold
            continue;

        const int eid = model->edgeId(vi, vj);
        if ( _edgeFaceSums.count(eid) > 0)  // Don't recalculate if already present
            continue;

        double esum = 0;
        for ( int fid : model->spolys( eid))
        {
            if ( mfids.count( fid) > 0) // Only count the faces in the manifold
                esum += _omcm->faceArea(fid);
        }   // end for

        _edgeFaceSums[eid] = esum;   // Needs normalising
    }   // end for
}   // end setEdgeFaceSums


// private
void ObjModelCurvatureMap::ManifoldData::setVertexAdjFaceSums( int j, int vi)
{
    const IntSet& mfids = _manf.manifold(j)->polygons();
    const ObjModel* model = _manf.cmodel();
    double fsum = 0;
    for ( int fid : model->faces( vi))
    {
        if ( mfids.count( fid) > 0) // Only count the faces in the manifold
            fsum += _omcm->faceArea(fid);
    }   // end for
    //assert( fsum > 0.0);
    _vtxAdjFacesSum[vi] = fsum;
}   // end setVertexAdjFaceSums


// private
void ObjModelCurvatureMap::ManifoldData::setVertexCurvature( int j, int vi)
{
    const ObjModel* model = _manf.cmodel();

    cv::Matx33d M( 0, 0, 0,
                   0, 0, 0,
                   0, 0, 0);

    const IntSet& mvids = _manf.manifold(j)->vertices();
    assert( mvids.count(vi) > 0);
    const IntSet& cvtxs = model->cvtxs(vi);
    for ( int vj : cvtxs)
    {
        if ( mvids.count(vj) > 0)   // Only incorporate curvature for edges in the manifold
            addEdgeCurvature( j, vi, vj, M);
    }   // end for

    const cv::Vec3d& N = _omcm->weightedVertexNormal( j, vi);
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
    // First column of Q is N (or -N depending on above +/- conditional).
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
void ObjModelCurvatureMap::ManifoldData::addEdgeCurvature( int j, int vi, int vj, cv::Matx33d& M)
{
    const cv::Vec3d& N = _omcm->weightedVertexNormal( j, vi);   // Normal to the surface at vi

    static const cv::Matx33d I( 1, 0, 0,
                                0, 1, 0,
                                0, 0, 1);

    const ObjModel* model = _manf.cmodel();
    const cv::Vec3f& ui = model->vtx(vi);
    const cv::Vec3f& uj = model->vtx(vj);
    const cv::Vec3d uji( uj[0] - ui[0], uj[1] - ui[1], uj[2] - ui[2]);    // uj - ui

    // Calc Tij as the unit vector in the tangent plane to the surface at point vi
    const cv::Matx33d nmat = I - N*N.t();
    cv::Vec3d Tij;
    cv::normalize( nmat * (-uji), Tij);

    double nuji = cv::norm(uji);
    double k = 0.0;
    if ( nuji > 0.0)
        k = 2.0 * N.dot( uji) / nuji; // Approximate the directional curvature

    // Calc edge weight ensuring that sum of the edge weights for all vertices connected to vi == 1
    double w = 0.0;
    if ( _vtxAdjFacesSum.at(vi) > 0.0)
        w = _edgeFaceSums.at( model->edgeId(vi,vj)) / _vtxAdjFacesSum.at(vi);

    M += w * k * Tij * Tij.t(); // Add the weighted curvature matrix
}   // end addEdgeCurvature


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

