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

#ifndef RFEATURES_OBJ_MODEL_CURVATURE_METRICS_H
#define RFEATURES_OBJ_MODEL_CURVATURE_METRICS_H

#include "ObjModelCurvatureMap.h"

namespace RFeatures {

class rFeatures_EXPORT ObjModelCurvatureMetrics
{
public:
    explicit ObjModelCurvatureMetrics( const ObjModelCurvatureMap&);

    // The first order derivative of the surface function is simply the curvature function.
    double faceKP1FirstOrder( int fid) const; // Max curvature
    double faceKP2FirstOrder( int fid) const; // Min curvature

    // The second order derivative of the surface function is the derivative of the curvature function.
    double faceKP1SecondOrder( int fid) const; // Max curvature
    double faceKP2SecondOrder( int fid) const; // Min curvature

    double faceDeterminant( int fid) const;

private:
    const ObjModelCurvatureMap& _cmap;
};  // end class

}   // end namespace

#endif
