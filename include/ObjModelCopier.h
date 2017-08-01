#ifndef RFEATURES_OBJ_MODEL_COPIER_H
#define RFEATURES_OBJ_MODEL_COPIER_H

/**
 * Copy a source model only from the parsed triangles with optional moving of vertices.
 */

#include "ObjModelTriangleMeshParser.h"
#include "ObjModelMover.h"

namespace RFeatures
{

class rFeatures_EXPORT ObjModelCopier : public ObjModelTriangleParser
{
public:
    ObjModelCopier( const ObjModel::Ptr source, const ObjModelMover& mover=ObjModelMover());

    void setRotationMatrix( double radians, const cv::Vec3d& axis);
    void setRotationMatrix( const cv::Matx33d& rotMat);

    void reset();

    ObjModel::Ptr getCopiedModel() const { return _cmodel;}

protected:
    virtual void parseTriangle( int fid, int uvroot, int uva, int uvb);

private:
    const ObjModel::Ptr _model;
    const ObjModelMover _mover;
    ObjModel::Ptr _cmodel;
};  // end class

}   // end namespace

#endif
