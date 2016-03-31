#pragma once
#ifndef RFEATURES_OBJ_MODEL_CLEANER_H
#define RFEATURES_OBJ_MODEL_CLEANER_H

#include "ObjModel.h"
#include "ObjModelTools.h"

namespace RFeatures
{

class rFeatures_EXPORT ObjModelCleaner
{
public:
    explicit ObjModelCleaner( ObjModel::Ptr model);

    ObjModel::Ptr getObjModel() const { return _model;}

    // Returns the number of unique vertices in the model that have no connections
    // to any other vertices in the model.
    size_t getNumLonelyVertices() const;

    // Remove all "tips" and "junction" vertices from the model.
    void removeTipsAndJunctions();

    // Find and remove faces that cause non-flat vertices, and
    // vertices that are unnecessary because all of their connected
    // vertices are non-flat.
    void removeNonFlat();

    // All vertices (and associated faces) from the model are removed
    // where they connect to fewer than minVtxFaceConns. Returns # of removed vertices.
    int pruneVertices( int minVtxFaceConns=3);

    // Remove vertices and adjoining faces on the boundary where the vertex
    // has edges greater than u + 0.6s where u is the mean edge length and s is
    // the standard deviation.
    void removeBoundarySpikes();

    // All vertices (and associated edges) are removed having an absolute
    // gradient at the vertex more than the specified amount. The hole is
    // filled using a replacement vertex that interpolates between the
    // old connected vertices.
    void adjustHighGradientVertices( double maxAbsGradient);

    // Given the provided unique vertices, remove them (and their attached faces)
    // from the model. Returns the modified model.
    void removeUniqueVertices( const IntSet& uvidxs);

    // Find and fill any holes in the model - returns number of holes filled.
    int fillHoles();

private:
    ObjModel::Ptr _model;
};  // end class

}   // end namespace

#endif
