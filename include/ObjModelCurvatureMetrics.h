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

namespace RFeatures {

class rFeatures_EXPORT ObjModelCurvatureMetrics
{
public:
    explicit ObjModelCurvatureMetrics( const ObjModelCurvatureMap*);
    virtual ~ObjModelCurvatureMetrics();

    const ObjModel* model() const { return _model;}

    // The first order derivative of the surface function is simply the curvature function.
    double faceKP1FirstOrder( int fid) const; // Max curvature
    double faceKP2FirstOrder( int fid) const; // Min curvature

    // The second order derivative of the surface function is the derivative of the curvature function.
    double faceKP1SecondOrder( int fid) const; // Max curvature
    double faceKP2SecondOrder( int fid) const; // Min curvature

    // The third order derivative of the surface function is the second derivative of the curvature function.
    double faceKP1ThirdOrder( int fid) const; // Max curvature
    double faceKP2ThirdOrder( int fid) const; // Min curvature

    double faceDeterminant( int fid) const;

private:
    const ObjModel* _model;
    std::unordered_map<int, IntSet>* _faceAdjFaces;

    std::unordered_map<int, double>* _faceMaxCurv0;  // Kp1
    std::unordered_map<int, double>* _faceMinCurv0;  // Kp2
    void calcFaceMaxCurvature0( const ObjModelCurvatureMap*, int);
    void calcFaceMinCurvature0( const ObjModelCurvatureMap*, int);

    std::unordered_map<int, double>* _faceMaxCurv1;  // Kp1
    std::unordered_map<int, double>* _faceMinCurv1;  // Kp2
    void calcFaceMaxCurvature1(int);
    void calcFaceMinCurvature1(int);

    std::unordered_map<int, double>* _faceMaxCurv2;  // Kp1
    std::unordered_map<int, double>* _faceMinCurv2;  // Kp2
    void calcFaceMaxCurvature2(int);
    void calcFaceMinCurvature2(int);

    std::unordered_map<int, double>* _faceDeterminants;
    void calcFaceDeterminant( const ObjModelCurvatureMap*, int);

    ObjModelCurvatureMetrics( const ObjModelCurvatureMetrics&) = delete;
    void operator=( const ObjModelCurvatureMetrics&) = delete;
};  // end class

}   // end namespace

#endif
