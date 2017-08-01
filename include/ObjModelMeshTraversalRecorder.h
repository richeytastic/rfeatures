#ifndef RFEATURES_OBJ_MODEL_MESH_TRAVERSAL_RECORDER_H
#define RFEATURES_OBJ_MODEL_MESH_TRAVERSAL_RECORDER_H

#include "ObjModelTriangleMeshParser.h"

namespace RFeatures
{

class rFeatures_EXPORT ObjModelMeshTraversalRecorder : public ObjModelTriangleParser
{
public:
    void reset();
    const IntSet& getTraversedVertices() const { return _uvidxs;}

protected:
    virtual void parseTriangle( int fid, int uvroot, int uva, int uvb);

private:
    IntSet _uvidxs;
};  // end class

}   // end namespace

#endif
