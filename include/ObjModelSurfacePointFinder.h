/**
 * A linear search for points on the surface of a model starting from some arbitrary vertex.
 * Richard Palmer
 * July 2017
 */

#ifndef RFEATURES_OBJ_MODEL_SURFACE_POINT_FINDER_H
#define RFEATURES_OBJ_MODEL_SURFACE_POINT_FINDER_H

#include "ObjModel.h"

namespace RFeatures
{

class rFeatures_EXPORT ObjModelSurfacePointFinder
{
public:
    ObjModelSurfacePointFinder( const ObjModel::Ptr);

    const ObjModel::Ptr getObject() const { return _model;}

    // Finds the point fv on the surface of the model closest to input vertex v.
    // On return, point fv will either be in the plane of one of the polygons attached to
    // vidx - in which case bfid will be set to the index of this face and vidx will be -1,
    // or fv will be in the same position as vertex vidx - in which case vidx will be unchanged
    // (as the input vertex) and bfid will be set to -1.
    // The starting input vertex vidx must be given.
    // Parameters bfid and fv may be set to anything (their correct values will be set on return).
    // Returns the squared l2-norm of (fv-v).
    double find( const cv::Vec3f& v, int& vidx, int& bfid, cv::Vec3f& fv) const;

private:
    const ObjModel::Ptr _model;
};  // end class

}   // end namespace

#endif
