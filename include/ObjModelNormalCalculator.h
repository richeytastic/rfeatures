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

#ifndef RFEATURES_OBJ_MODEL_NORMAL_CALCULATOR_H
#define RFEATURES_OBJ_MODEL_NORMAL_CALCULATOR_H

#include "ObjModelTriangleMeshParser.h"

namespace RFeatures
{

class rFeatures_EXPORT ObjModelNormalCalculator : public ObjModelTriangleParser
{
public:
    explicit ObjModelNormalCalculator( const ObjModel::Ptr);

    const ObjModel::Ptr getObject() const { return _model;}

    void reset();

    // Calculate and return the polygon normal given the three vertex IDs root, a, and b.
    // The normal is calculated according to the right hand rule (va-vroot) x (vb-vroot)
    // (where x is the cross product). Returned vector is normalised.
    cv::Vec3d operator()( int root, int a, int b) const;

    // Static version of above for arbitrary set of vertices (don't need to be an existing face!).
    static cv::Vec3d calcNormal( const ObjModel::Ptr, int root, int a, int b);

    // Static version of normal calculation. If the referenced face has texture UVs,
    // the ordering of these is used to generate the surface normal for that face, otherwise
    // the direction of the normal is according to the underlying storage of the face vertices
    // (which are stored in ascending order of vertex ID).
    static cv::Vec3d calcNormal( const ObjModel::Ptr, int fid);

    // Recalculate and return the polygon normal on the underlying model.
    // (useful if the model has changed externally and don't want to redo whole model).
    // This function will work even if polygon has not been calculated previously BUT
    // a polygon that shares this face MUST have been calculated previously since the
    // direction of the normals must be consistent between adjacent polygons.
    const cv::Vec3d& recalcFaceNormal( int fid);

    // These functions valid after parsing through ObjModelTriangleMeshParser
    const cv::Vec3d& getFaceNormal( int fid) const { return _faceNormals.at(fid);}
    const cv::Vec3i& getFaceVtxOrder( int fid) const { return _faceVtxOrder.at(fid);}

    bool isPresent( int fid) const { return _faceNormals.count(fid) > 0;}

    void remove( int fid);  // Remove information about this polygon.

protected:
    virtual void parseTriangle( int fid, int uvroot, int uva, int uvb);

private:
    const ObjModel::Ptr _model;
    boost::unordered_map<int, cv::Vec3d> _faceNormals;
    boost::unordered_map<int, cv::Vec3i> _faceVtxOrder; // faceId-->cv::Vec3i(root, a, b)
};  // end class

}   // end namespace

#endif
