#ifndef RFEATURES_OBJ_MODEL_VERTEX_ADDER_H
#define RFEATURES_OBJ_MODEL_VERTEX_ADDER_H

#include "ObjModel.h"

namespace RFeatures
{

class rFeatures_EXPORT ObjModelVertexAdder
{
public:
    // Adds a bunch more extra vertices to edges that are longer than minDistance.
    // On return, the provided model will not have any triangle edges longer than
    // minDistance. Apart from subdividing the model's triangles (and adding more
    // complexity), this does nothing to change the morphology of the model.
    // Returns the number of new vertices added.
    static int addVertices( ObjModel::Ptr, double minDistance);

};  // end class

}   // end namespace

#endif
