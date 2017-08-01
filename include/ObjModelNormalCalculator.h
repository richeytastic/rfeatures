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

    // Recalculate and return the polygon normal on the underlying model.
    // (useful if the model has changed externally and don't want to redo whole model).
    // This function will work even if polygon has not been calculated previously BUT
    // a polygon that shares this face MUST have been calculated previously since the
    // direction of the normals must be consistent between adjacent polygons.
    const cv::Vec3d& recalcFaceNormal( int fid);

    // These functions valid after parsing through ObjModelTriangleMeshParser
    const cv::Vec3d& getFaceNormal( int fid) const { return _faceNormals.at(fid);}
    const cv::Vec3i& getFaceVtxOrder( int fid) const { return _faceVtxOrder.at(fid);}

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
