#ifndef RFEATURES_OBJ_MODEL_CROPPER_H
#define RFEATURES_OBJ_MODEL_CROPPER_H

#include "ObjModelTriangleMeshParser.h"

namespace RFeatures
{

class rFeatures_EXPORT ObjModelCropper : public RFeatures::ObjModelBoundaryParser
{
public:
    ObjModelCropper( const ObjModel::Ptr m, const cv::Vec3f& originVertex, double radiusThreshold);

protected:
    virtual bool parseEdge( int fid, int vid0, int vid1);

private:
    const ObjModel::Ptr _m;
    const cv::Vec3f _ov;
    double _sqRadiusThreshold;
};  // end class


}   // end namespace

#endif
