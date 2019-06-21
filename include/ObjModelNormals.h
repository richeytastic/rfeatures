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

#ifndef RFEATURES_OBJ_MODEL_NORMALS_H
#define RFEATURES_OBJ_MODEL_NORMALS_H

#include "ObjModelTriangleMeshParser.h"

namespace RFeatures {

class rFeatures_EXPORT ObjModelNormals : public ObjModelTriangleParser
{
public:
    ObjModelNormals();

    void reset() override;

    // Calculate and return the polygon normal given the three vertex IDs root, a, and b.
    // The normal is calculated according to the right hand rule (va-vroot) x (vb-vroot)
    // (where x is the cross product). Returned vector is normalised.
    static cv::Vec3d calcNormal( const ObjModel&, int root, int a, int b);

    // If the referenced face has texture UVs, the ordering of these is used to generate the
    // surface normal for that face, otherwise the direction of the normal is according to
    // the underlying storage of the face vertices (which are in ascending order of vertex ID).
    static cv::Vec3d calcNormal( const ObjModel&, int fid);

    // Recalculate, set in this object and return the polygon normal on the underlying model.
    // Call this function if model has changed and don't want to reparse the whole model.
    // Works even if normal for polygon has not been calculated previously but in this case
    // a polygon that shares this face MUST have had its normal already calculated since the
    // direction of the normal for the adjacent polygon is used to determine the surface
    // orientation (and keep consistent surface orientation across all polygons).
    const cv::Vec3d& recalcFaceNormal( int fid);

    // These functions valid after parsing through ObjModelTriangleMeshParser
    const cv::Vec3d& normal( int fid) const { return _faceNormals.at(fid);}
    const cv::Vec3i& getFaceVtxOrder( int fid) const { return _faceVtxOrder.at(fid);}

    bool isPresent( int fid) const { return _faceNormals.count(fid) > 0;}
    void remove( int fid);  // Remove information about this polygon.

protected:
    void parseTriangle( int fid, int uvroot, int uva, int uvb) override;

private:
    std::unordered_map<int, cv::Vec3d> _faceNormals;
    std::unordered_map<int, cv::Vec3i> _faceVtxOrder; // faceId-->cv::Vec3i(root, a, b)

    ObjModelNormals( const ObjModelNormals&) = delete;
    void operator=( const ObjModelNormals&) = delete;
};  // end class

}   // end namespace

#endif
