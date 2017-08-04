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

#include "ObjModelCurvatureMetrics.h"
using RFeatures::ObjModelCurvatureMetrics;
using RFeatures::ObjModelCurvatureMap;
using RFeatures::ObjModel;
#include <boost/foreach.hpp>


class ObjModelCurvatureMetrics::Deleter
{ public:
    void operator()( ObjModelCurvatureMetrics* m) { delete m;}
};  // end class


// public static
ObjModelCurvatureMetrics::Ptr ObjModelCurvatureMetrics::create( const ObjModelCurvatureMap::Ptr cm)
{
    return Ptr( new ObjModelCurvatureMetrics( cm), Deleter());
}   // end create



// private
ObjModelCurvatureMetrics::ObjModelCurvatureMetrics( const ObjModelCurvatureMap::Ptr cm) : _curvMap(cm)
{
    _faceAdjFaces = new boost::unordered_map<int,IntSet>;
    _faceDeterminants = new boost::unordered_map<int,double>;
    _faceMaxCurv0 = new boost::unordered_map<int,double>;
    _faceMaxCurv1 = new boost::unordered_map<int,double>;
    _faceMaxCurv2 = new boost::unordered_map<int,double>;
    _faceMinCurv0 = new boost::unordered_map<int,double>;
    _faceMinCurv1 = new boost::unordered_map<int,double>;
    _faceMinCurv2 = new boost::unordered_map<int,double>;

    const ObjModel::Ptr model = _curvMap->getObject();
    const IntSet& fids = model->getFaceIds();
    BOOST_FOREACH ( int fid, fids)
    {
        calcFaceDeterminant( fid);
        calcFaceMinCurvature0( fid);
        calcFaceMaxCurvature0( fid);
    }   // end foreach

    // Get each face's adjacent faces
    BOOST_FOREACH ( int fid, fids)
        model->findAdjacentFaces( fid, (*_faceAdjFaces)[fid]);

    // Calc the first derivative of curvature
    BOOST_FOREACH ( int fid, fids)
    {
        calcFaceMinCurvature1( fid);
        calcFaceMaxCurvature1( fid);
    }   // end foreach

    // Calc the second derivative of curvature
    BOOST_FOREACH ( int fid, fids)
    {
        calcFaceMinCurvature2( fid);
        calcFaceMaxCurvature2( fid);
    }   // end foreach

    delete _faceAdjFaces;
}   // end ctor


// private
ObjModelCurvatureMetrics::~ObjModelCurvatureMetrics()
{
    delete _faceDeterminants;
    delete _faceMaxCurv0;
    delete _faceMinCurv0;
    delete _faceMaxCurv1;
    delete _faceMinCurv1;
    delete _faceMaxCurv2;
    delete _faceMinCurv2;
}   // end dtor


// public
double ObjModelCurvatureMetrics::getFaceDeterminant( int fid) const
{
    return _faceDeterminants->at(fid);
}   // end getFaceDeterminant


// public
double ObjModelCurvatureMetrics::getFaceKP1FirstOrder( int fid) const
{
    return _faceMaxCurv0->at(fid);
}   // end getFaceKP1FirstOrder


// public
double ObjModelCurvatureMetrics::getFaceKP2FirstOrder( int fid) const
{
    return _faceMinCurv0->at(fid);
}   // end getFaceKP2FirstOrder


// public
double ObjModelCurvatureMetrics::getFaceKP1SecondOrder( int fid) const
{
    return _faceMaxCurv1->at(fid);
}   // end getFaceKP1SecondOrder


// public
double ObjModelCurvatureMetrics::getFaceKP2SecondOrder( int fid) const
{
    return _faceMinCurv1->at(fid);
}   // end getFaceKP2SecondOrder


// public
double ObjModelCurvatureMetrics::getFaceKP1ThirdOrder( int fid) const
{
    return _faceMaxCurv2->at(fid);
}   // end getFaceKP1ThirdOrder


// public
double ObjModelCurvatureMetrics::getFaceKP2ThirdOrder( int fid) const
{
    return _faceMinCurv2->at(fid);
}   // end getFaceKP2ThirdOrder


// private
void ObjModelCurvatureMetrics::calcFaceDeterminant( int fid)
{
    const ObjPoly& face = _curvMap->getObject()->getFace( fid);
    const cv::Vec3d& n0 = _curvMap->getVertexNormal( face.vindices[0]);
    const cv::Vec3d& n1 = _curvMap->getVertexNormal( face.vindices[1]);
    const cv::Vec3d& n2 = _curvMap->getVertexNormal( face.vindices[2]);
    (*_faceDeterminants)[fid] = n2.dot(n0.cross(n1));   // Calculate determinant as the scalar triple product
}   // end calcFaceDeterminant


// private
void ObjModelCurvatureMetrics::calcFaceMaxCurvature0( int fid)
{
    const ObjPoly& face = _curvMap->getObject()->getFace( fid);
    double ka, kb, kc;
    _curvMap->getVertexPrincipalCurvature1( face.vindices[0], ka);
    _curvMap->getVertexPrincipalCurvature1( face.vindices[1], kb);
    _curvMap->getVertexPrincipalCurvature1( face.vindices[2], kc);
    // Face curvature is the average of the curvature at the 3 corner vertices. Note that these
    // curvatures have already been calculated using weights corresponding to the relative area
    // of this polygon with the sum of the area of the polygons connected to each of the vertices.
    (*_faceMaxCurv0)[fid] = (ka + kb + kc)/3;
}   // end calcFaceMaxCurvature0


// private
void ObjModelCurvatureMetrics::calcFaceMaxCurvature1( int fid)
{
    // The derivative of curvature is the difference in curvature between this face's
    // curvature and the curvature of its (up to) three adjacent neighbours.
    const double k = _faceMaxCurv0->at(fid);
    const ObjModel::Ptr model = _curvMap->getObject();
    double fdiff = 0.0;
    const IntSet& fset = _faceAdjFaces->at(fid);
    BOOST_FOREACH ( int fsid, fset)
        fdiff += k - _faceMaxCurv0->at(fsid);
    (*_faceMaxCurv1)[fid] = fdiff/fset.size();
}   // end calcFaceMaxCurvature1

// private
void ObjModelCurvatureMetrics::calcFaceMaxCurvature2( int fid)
{
    const double k = _faceMaxCurv1->at(fid);
    const ObjModel::Ptr model = _curvMap->getObject();
    double fdiff = 0.0;
    const IntSet& fset = _faceAdjFaces->at(fid);
    BOOST_FOREACH ( int fsid, fset)
        fdiff += k - _faceMaxCurv1->at(fsid);
    (*_faceMaxCurv2)[fid] = fdiff/fset.size();
}   // end calcFaceMaxCurvature2



// private
void ObjModelCurvatureMetrics::calcFaceMinCurvature0( int fid)
{
    const ObjPoly& face = _curvMap->getObject()->getFace( fid);
    double ka, kb, kc;
    _curvMap->getVertexPrincipalCurvature2( face.vindices[0], ka);
    _curvMap->getVertexPrincipalCurvature2( face.vindices[1], kb);
    _curvMap->getVertexPrincipalCurvature2( face.vindices[2], kc);
    // Face curvature is the average of the curvature at the 3 corner vertices. Note that these
    // curvatures have already been calculated using weights corresponding to the relative area
    // of this polygon with the sum of the area of the polygons connected to each of the vertices.
    (*_faceMinCurv0)[fid] = (ka + kb + kc)/3;
}   // end calcFaceMinCurvature0

// private
void ObjModelCurvatureMetrics::calcFaceMinCurvature1( int fid)
{
    // The derivative of curvature is the difference in curvature between this face's
    // curvature and the curvature of its (up to) three adjacent neighbours.
    const double k = _faceMinCurv0->at(fid);
    const ObjModel::Ptr model = _curvMap->getObject();
    double fdiff = 0.0;
    const IntSet& fset = _faceAdjFaces->at(fid);
    BOOST_FOREACH ( int fsid, fset)
        fdiff += k - _faceMinCurv0->at(fsid);
    (*_faceMinCurv1)[fid] = fdiff/fset.size();
}   // end calcFaceMinCurvature1

// private
void ObjModelCurvatureMetrics::calcFaceMinCurvature2( int fid)
{
    const double k = _faceMinCurv1->at(fid);
    const ObjModel::Ptr model = _curvMap->getObject();
    double fdiff = 0.0;
    const IntSet& fset = _faceAdjFaces->at(fid);
    BOOST_FOREACH ( int fsid, fset)
        fdiff += k - _faceMinCurv1->at(fsid);
    (*_faceMinCurv2)[fid] = fdiff/fset.size();
}   // end calcFaceMinCurvature2

