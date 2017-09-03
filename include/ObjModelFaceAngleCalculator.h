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

#ifndef RFEATURES_OBJ_MODEL_FACE_ANGLE_CALCULATOR_H
#define RFEATURES_OBJ_MODEL_FACE_ANGLE_CALCULATOR_H

/**
 * Calculates the inner angles of the polygonal faces of ObjModels.
 */
#include "ObjModelTriangleMeshParser.h"

namespace RFeatures
{

typedef boost::unordered_map<int, double> VertexAngles; // Inner angles of vertices within a face
typedef boost::unordered_map<int, VertexAngles> FaceAngles;


class rFeatures_EXPORT ObjModelFaceAngleCalculator : public ObjModelTriangleParser
{
public:
    ObjModelFaceAngleCalculator();

    void reset();

    // Calculate the inner angle at v0 of the triangle defined by the three vertices.
    static double calcAngle( const cv::Vec3f& v0, const cv::Vec3f& v1, const cv::Vec3f& v2);

    void calcFaceAngles( int fid); // Face angles not updated until this function called.
    const FaceAngles& getFaceAngles() const { return _faces;}

    // Return the inner angle of vidx inside triangle fid.
    // Returns negative if vidx not in referenced face.
    double calcInnerAngle( int fid, int vidx) const;

    // Static version of above function.
    static double calcInnerAngle( const ObjModel::Ptr, int fid, int vidx);

protected:
    virtual void parseTriangle( int fid, int vroot, int va, int vb);

private:
    FaceAngles _faces;
    ObjModelFaceAngleCalculator( const ObjModelFaceAngleCalculator&);
    void operator=( const ObjModelFaceAngleCalculator&);
};  // end class

}   // end namespace

#endif
