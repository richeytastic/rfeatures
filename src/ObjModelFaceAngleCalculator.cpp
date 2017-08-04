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

#include "ObjModelFaceAngleCalculator.h"
using RFeatures::ObjModelFaceAngleCalculator;
using RFeatures::ObjModel;
using RFeatures::ObjPoly;
#include <cassert>
#include <cmath>

 
ObjModelFaceAngleCalculator::ObjModelFaceAngleCalculator( const ObjModel::Ptr m) : _model(m)
{
}   // end ctor


void ObjModelFaceAngleCalculator::reset()
{
    _faces.clear();
}   // end reset


// public static
double ObjModelFaceAngleCalculator::calcAngle( const cv::Vec3f& v, const cv::Vec3f& v0, const cv::Vec3f& v1)
{
    const cv::Vec3d u0 = v0 - v;
    const cv::Vec3d u1 = v1 - v;
    return acos( u0.dot(u1) / (cv::norm(u0) * cv::norm(u1)));
}   // end calcAngle


// public
void ObjModelFaceAngleCalculator::calcFaceAngles( int fid)
{
    const ObjPoly& face = _model->getFace( fid);
    parseTriangle( fid, face.vindices[0], face.vindices[1], face.vindices[2]);
}   // end calctFaceAngles


// protected
void ObjModelFaceAngleCalculator::parseTriangle( int fid, int u0, int u1, int u2)
{
    _faces[fid][u0] = calcAngle( _model->getVertex( u0), _model->getVertex( u1), _model->getVertex( u2));
    _faces[fid][u1] = calcAngle( _model->getVertex( u1), _model->getVertex( u0), _model->getVertex( u2));
    _faces[fid][u2] = calcAngle( _model->getVertex( u2), _model->getVertex( u0), _model->getVertex( u1));
}   // end parseTriangle


// public
double ObjModelFaceAngleCalculator::operator()( int fid, int u0) const
{
    const ObjPoly& face = _model->getFace( fid);
    int u1, u2;
    if ( !face.getOpposite( u0, u1, u2))
        return -1;
    const cv::Vec3f& v0 = _model->getVertex( u0);
    const cv::Vec3f& v1 = _model->getVertex( u1);
    const cv::Vec3f& v2 = _model->getVertex( u2);
    return calcAngle( v0, v1, v2);
}   // end 
