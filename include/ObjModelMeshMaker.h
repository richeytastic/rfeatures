#ifndef RFEATURES_OBJ_MODEL_MESH_MAKER_H
#define RFEATURES_OBJ_MODEL_MESH_MAKER_H

#include "ObjModel.h"

namespace RFeatures
{

class ObjModelMeshMaker
{
public:
    explicit ObjModelMeshMaker( ObjModel::Ptr);

    // Discovers and builds a mesh on the object returning the number of polygons created.
    // If the object already has polygons, nothing happens and that number of polygons is returned.
    int operator()();

private:
    ObjModel::Ptr _model;
};  // end class

}   // end namespace

#endif
