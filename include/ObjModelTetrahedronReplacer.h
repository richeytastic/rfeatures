/**
 * Replace tetrahedrons in the model. Only works with triangulated meshes!
 */

#ifndef RFEATURES_OBJ_MODEL_TETRAHEDRON_REPLACER_H
#define RFEATURES_OBJ_MODEL_TETRAHEDRON_REPLACER_H

#include "ObjModel.h"

namespace RFeatures
{

class rFeatures_EXPORT ObjModelTetrahedronReplacer
{
public:
    ObjModelTetrahedronReplacer( ObjModel::Ptr);

    // Finds vertices that are the points of three adjacent polygons and removes these polygons
    // replacing them with a single triangle (if one does not already exist). Returns the number
    // of vertices removed.
    int removeTetrahedrons();

private:
    ObjModel::Ptr _model;
};  // end class

}   // end namespace

#endif

