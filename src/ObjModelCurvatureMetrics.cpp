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

#include <ObjModelCurvatureMetrics.h>
using RFeatures::ObjModelCurvatureMetrics;
typedef RFeatures::ObjModelCurvatureMap OMCM;


// public
ObjModelCurvatureMetrics::ObjModelCurvatureMetrics( const OMCM* cm) : _model(cm->model())
{
    using std::unordered_map;
    _faceAdjFaces = new unordered_map<int,IntSet>;
    _faceMaxCurv0 = new unordered_map<int,double>;
    _faceMaxCurv1 = new unordered_map<int,double>;
    _faceMaxCurv2 = new unordered_map<int,double>;
    _faceMinCurv0 = new unordered_map<int,double>;
    _faceMinCurv1 = new unordered_map<int,double>;
    _faceMinCurv2 = new unordered_map<int,double>;
    _faceDeterminants = new unordered_map<int,double>;

    const IntSet& fids = _model->getFaceIds();

    for ( int fid : fids)
    {
        calcFaceDeterminant( cm, fid);
        calcFaceMinCurvature0( cm, fid);
        calcFaceMaxCurvature0( cm, fid);
    }   // end foreach

    // Get each face's adjacent faces
    for ( int fid : fids)
        _model->findAdjacentFaces( fid, (*_faceAdjFaces)[fid]);

    // Calc the first derivative of curvature
    for ( int fid : fids)
    {
        calcFaceMinCurvature1( fid);
        calcFaceMaxCurvature1( fid);
    }   // end foreach

    // Calc the second derivative of curvature
    for ( int fid : fids)
    {
        calcFaceMinCurvature2( fid);
        calcFaceMaxCurvature2( fid);
    }   // end foreach

    delete _faceAdjFaces;
}   // end ctor


// public
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
double ObjModelCurvatureMetrics::faceDeterminant( int fid) const    { return _faceDeterminants->at(fid);}
double ObjModelCurvatureMetrics::faceKP1FirstOrder( int fid) const  { return _faceMaxCurv0->at(fid);}
double ObjModelCurvatureMetrics::faceKP2FirstOrder( int fid) const  { return _faceMinCurv0->at(fid);}
double ObjModelCurvatureMetrics::faceKP1SecondOrder( int fid) const { return _faceMaxCurv1->at(fid);}
double ObjModelCurvatureMetrics::faceKP2SecondOrder( int fid) const { return _faceMinCurv1->at(fid);}
double ObjModelCurvatureMetrics::faceKP1ThirdOrder( int fid) const  { return _faceMaxCurv2->at(fid);}
double ObjModelCurvatureMetrics::faceKP2ThirdOrder( int fid) const  { return _faceMinCurv2->at(fid);}


// private
void ObjModelCurvatureMetrics::calcFaceDeterminant( const OMCM* cm, int fid)
{
    const int* vindices = _model->getFaceVertices( fid);
    const cv::Vec3d& n0 = cm->weightedVertexNormal( vindices[0]);
    const cv::Vec3d& n1 = cm->weightedVertexNormal( vindices[1]);
    const cv::Vec3d& n2 = cm->weightedVertexNormal( vindices[2]);
    (*_faceDeterminants)[fid] = n2.dot(n0.cross(n1));   // Calculate determinant as the scalar triple product
}   // end calcFaceDeterminant


// private
void ObjModelCurvatureMetrics::calcFaceMaxCurvature0( const OMCM* cm, int fid)
{
    const int* vindices = _model->getFaceVertices( fid);
    double ka, kb, kc;
    cm->vertexPC1( vindices[0], ka);
    cm->vertexPC1( vindices[1], kb);
    cm->vertexPC1( vindices[2], kc);
    // Face curvature is the average of the curvature at the 3 corner vertices. Note that these
    // curvatures have already been calculated using weights corresponding to the relative area
    // of this polygon with the sum of the area of the polygons connected to each of the vertices.
    (*_faceMaxCurv0)[fid] = (ka + kb + kc)/3;
}   // end calcFaceMaxCurvature0


// private
void ObjModelCurvatureMetrics::calcFaceMinCurvature0( const OMCM* cm, int fid)
{
    const int* vindices = _model->getFaceVertices( fid);
    double ka, kb, kc;
    cm->vertexPC2( vindices[0], ka);
    cm->vertexPC2( vindices[1], kb);
    cm->vertexPC2( vindices[2], kc);
    // Face curvature is the average of the curvature at the 3 corner vertices. Note that these
    // curvatures have already been calculated using weights corresponding to the relative area
    // of this polygon with the sum of the area of the polygons connected to each of the vertices.
    (*_faceMinCurv0)[fid] = (ka + kb + kc)/3;
}   // end calcFaceMinCurvature0


// private
void ObjModelCurvatureMetrics::calcFaceMaxCurvature1( int fid)
{
    // The derivative of curvature is the difference in curvature between this face's
    // curvature and the curvature of its (up to) three adjacent neighbours.
    const double k = _faceMaxCurv0->at(fid);
    double fdiff = 0.0;
    const IntSet& fset = _faceAdjFaces->at(fid);
    for ( int fsid : fset)
        fdiff += k - _faceMaxCurv0->at(fsid);
    (*_faceMaxCurv1)[fid] = fdiff/fset.size();
}   // end calcFaceMaxCurvature1


// private
void ObjModelCurvatureMetrics::calcFaceMaxCurvature2( int fid)
{
    const double k = _faceMaxCurv1->at(fid);
    double fdiff = 0.0;
    const IntSet& fset = _faceAdjFaces->at(fid);
    for ( int fsid : fset)
        fdiff += k - _faceMaxCurv1->at(fsid);
    (*_faceMaxCurv2)[fid] = fdiff/fset.size();
}   // end calcFaceMaxCurvature2


// private
void ObjModelCurvatureMetrics::calcFaceMinCurvature1( int fid)
{
    // The derivative of curvature is the difference in curvature between this face's
    // curvature and the curvature of its (up to) three adjacent neighbours.
    const double k = _faceMinCurv0->at(fid);
    double fdiff = 0.0;
    const IntSet& fset = _faceAdjFaces->at(fid);
    for ( int fsid : fset)
        fdiff += k - _faceMinCurv0->at(fsid);
    (*_faceMinCurv1)[fid] = fdiff/fset.size();
}   // end calcFaceMinCurvature1


// private
void ObjModelCurvatureMetrics::calcFaceMinCurvature2( int fid)
{
    const double k = _faceMinCurv1->at(fid);
    double fdiff = 0.0;
    const IntSet& fset = _faceAdjFaces->at(fid);
    for ( int fsid : fset)
        fdiff += k - _faceMinCurv1->at(fsid);
    (*_faceMinCurv2)[fid] = fdiff/fset.size();
}   // end calcFaceMinCurvature2
