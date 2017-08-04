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

#ifndef RFEATURES_OBJ_MODEL_CURVATURE_METRICS_H
#define RFEATURES_OBJ_MODEL_CURVATURE_METRICS_H

#include "ObjModelCurvatureMap.h"

namespace RFeatures
{

class rFeatures_EXPORT ObjModelCurvatureMetrics
{
public:
    typedef boost::shared_ptr<ObjModelCurvatureMetrics> Ptr;
    static Ptr create( const ObjModelCurvatureMap::Ptr);

    // Only the faces traversed by this curvature map have valid curvature calculated.
    // Accessing curvature information about other faces in the model will cause an error.
    const ObjModelCurvatureMap::Ptr getCurvatureMap() const { return _curvMap;}

    // The first order derivative of the surface function is simply the curvature function.
    double getFaceKP1FirstOrder( int fid) const; // Max curvature
    double getFaceKP2FirstOrder( int fid) const; // Min curvature

    // The second order derivative of the surface function is the derivative of the curvature function.
    double getFaceKP1SecondOrder( int fid) const; // Max curvature
    double getFaceKP2SecondOrder( int fid) const; // Min curvature

    // The third order derivative of the surface function is the second derivative of the curvature function.
    double getFaceKP1ThirdOrder( int fid) const; // Max curvature
    double getFaceKP2ThirdOrder( int fid) const; // Min curvature

    double getFaceDeterminant( int fid) const;

private:
    const ObjModelCurvatureMap::Ptr _curvMap;
    boost::unordered_map<int, IntSet>* _faceAdjFaces;

    boost::unordered_map<int, double>* _faceMaxCurv0;  // Kp1
    boost::unordered_map<int, double>* _faceMinCurv0;  // Kp2
    void calcFaceMaxCurvature0(int);
    void calcFaceMinCurvature0(int);

    boost::unordered_map<int, double>* _faceMaxCurv1;  // Kp1
    boost::unordered_map<int, double>* _faceMinCurv1;  // Kp2
    void calcFaceMaxCurvature1(int);
    void calcFaceMinCurvature1(int);

    boost::unordered_map<int, double>* _faceMaxCurv2;  // Kp1
    boost::unordered_map<int, double>* _faceMinCurv2;  // Kp2
    void calcFaceMaxCurvature2(int);
    void calcFaceMinCurvature2(int);

    boost::unordered_map<int, double>* _faceDeterminants;
    void calcFaceDeterminant(int);

    explicit ObjModelCurvatureMetrics( const ObjModelCurvatureMap::Ptr);
    virtual ~ObjModelCurvatureMetrics();
    class Deleter;
};  // end class

}   // end namespace

#endif
