/**
 * Collapse polygons in models that are triangulated meshes
 * without leaving holes - maintaining the model's triangulation.
 */

#ifndef RFEATURES_OBJ_MODEL_POLYGON_COLLAPSER_H
#define RFEATURES_OBJ_MODEL_POLYGON_COLLAPSER_H

#include "ObjModel.h"

namespace RFeatures
{

class rFeatures_EXPORT ObjModelPolygonCollapser
{
public:
    explicit ObjModelPolygonCollapser( ObjModel::Ptr);

    // Collapse the given polygon to a point (removing it in the process).
    // Maintains the model's triangulation. Returns the number of other polygons
    // shared by the collapsed polygon. Returns -1 if fid is not a valid polygon
    // and -2 if there is not a valid triangulation around the polygon.
    int collapse( int fid);

private:
    ObjModel::Ptr _model;
};  // end class

}   // end namespace

#endif
