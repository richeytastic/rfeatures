/************************************************************************
 * Copyright (C) 2017 Richard Palmer
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 ************************************************************************/

#ifndef RFEATURES_OBJ_MODEL_CLEANER_H
#define RFEATURES_OBJ_MODEL_CLEANER_H

#include "ObjModel.h"
#include <ProgressDelegate.h> // rlib

namespace RFeatures {

class rFeatures_EXPORT ObjModelCleaner
{
public:
    explicit ObjModelCleaner( ObjModel::Ptr originalToBeCleaned);
    virtual ~ObjModelCleaner();

    ObjModel::Ptr model() const { return _model;}

    size_t getNumLonelyVertices() const { return _lonely->size();}
    size_t getNumFlatVertices() const { return _flat->size();}
    size_t getNumNonFlatVertices() const { return _nonflat->size();}
    size_t getNumEdgeVertices() const { return _edge->size();}
    size_t getNumCompVertices() const { return _comp->size();}

    // Find and remove faces that cause tetrahedrons to arise.
    int remove3D();

    // Find and remove unconnected (lonely) vertices and vertices that are the sole
    // connection points of 2 (or more) connected polygons.
    int remove1D();

    // Remove vertices (and associated faces) where they connect to
    // fewer than minVtxFaceConns faces. Returns # of removed vertices.
    int pruneVertices( int minVtxFaceConns=3);

    // Remove the provided vertices and their attached faces from the model.
    void removeVertices( const IntSet& vidxs);

    void setProgressDelegate( rlib::ProgressDelegate*);

private:
    rlib::ProgressDelegate *_progressDelegate;
    ObjModel::Ptr _model;
    IntSet *_lonely, *_flat, *_nonflat, *_edge, *_comp;    // Sets for the different topologies
    void updateVertexTopology( int vidx);
    void regatherTopology();
    bool removeVertex( int vidx);  // Returns true if vidx was present for removal
    void removeFace( int fid);
    void removeVertexAndFaces( int vidx);
    bool is3DExtrusion( int vidx) const;
    void remove1DSet( const IntSet*);

    // A vertex x is extruded if for its set of connected vertices C_x, every edge formed by every
    // member of C_x and x is shared by at least three polygons. In this case, vertex x can be safely removed.
    int removeFlatExtrusions(); // Returns number of extrusions removed
    void removeTetrahedronBases();  // Same logic as removeFlatExtrusions but the offended polygon is removed instead.
    int removeNonFlatMakingEdges( int vidx);
    int removeJunctionConnections( int vidx); // Returns number of connected vertices removed
    int removeSurfaceJoin( int vidx);
};  // end class

}   // end namespace

#endif
