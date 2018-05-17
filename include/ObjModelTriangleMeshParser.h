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

#ifndef RFEATURES_OBJ_MODEL_TRIANGLE_MESH_PARSER_H
#define RFEATURES_OBJ_MODEL_TRIANGLE_MESH_PARSER_H

/**
 * Parses an object as a triangulated mesh with consistent ordering of
 * triangle vertices to allow for reliable calculation of surface
 * orientated metrics e.g. surface normals for all adjacent triangles.
 * The object must form a triangulated mesh (i.e. no edge can be
 * used by more than 2 triangles).
 */
#include "ObjModel.h"

namespace RFeatures {

// Parse the given triangle with a consistent vertex edge ordering.
// The vertices are always the vertex IDs in the associated model.
struct rFeatures_EXPORT ObjModelTriangleParser
{
    virtual void parseTriangle( int fid, int vroot, int va, int vb) = 0;
    virtual void finishedParsing() {}   // Be informed when parsing all triangles finished.
    const ObjModel* model;
    virtual void reset(){}  // Always called by ObjModelTriangleMeshParser after model set
};  // end struct

// Given a triangle edge in the direction v0->v1, specify that parsing should progress beyond this
// edge by returning true. Returning false causes the mesh parser to treat this edge as a boundary
// (whether or not it is an actual boundary of the mesh). It's safe to always return true from this
// function in which case traversal will continue up to the actual bounds of the mesh. The provided
// face ID is that of the triangle just parsed, and the vertices are the edge (either the top or
// the left edge of that face).
struct rFeatures_EXPORT ObjModelBoundaryParser
{
    virtual bool parseEdge( int fid, int v0, int v1) = 0;
    virtual void finishedParsing() {}   // Be informed when parsing all triangles finished.
    const ObjModel* model;
    virtual void reset(){}  // Always called by ObjModelTriangleMeshParser after model set
};  // end struct


class rFeatures_EXPORT ObjModelTriangleMeshParser
{
public:
    // If desired, set pfaces to point to a set that will collect all polygon IDs parsed
    // on the component (i.e. for a single call to parse). Caller MUST NOT alter the
    // contents of pfaces while the call to parse() is ongoing! Note that the contents
    // of pfaces is cleared at the beginning of every call to parse().
    ObjModelTriangleMeshParser( const ObjModel*, IntSet* pfaces=NULL);
    virtual ~ObjModelTriangleMeshParser();

    // Replace the existing face parse set with the given one. Only safe to call between calls to parse().
    // This set is cleared at the beginning of every call to parse(). Do not call with NULL!
    void setParseSet( IntSet*);

    // Only triangles connected via a shared edge are included in the parsing.
    // Returns number of triangles parsed or -1 if the model is found to have
    // edges with greater than 2 triangles.
    // startPoly: The ID of the model polygon to start parsing with.
    // planev:    The half of coordinate space to use as the "positive"
    //            half regarding the direction of the starting polygon's normal.
    //            This defines the consistent parse ordering of polygon vertices.
    //            If left as default (zero vector), an undefined ordering is used.
    int parse( int startPoly=-1, const cv::Vec3d planev=cv::Vec3d(0,0,0));

    // Add processing delegates
    void setBoundaryParser( ObjModelBoundaryParser*);
    bool addTriangleParser( ObjModelTriangleParser*);   // Returns true if added or already set

private:
    const ObjModel* _model;
    IntSet *_parsedFaces;
    bool _dodel;

    ObjModelBoundaryParser* _bparser;
    std::unordered_set<ObjModelTriangleParser*> _tparsers;

    void processTriangleParsers( int, int, int, int);
    void informFinishedParsing();
    bool parseEdge( int, int, int);
    struct Triangle;
};  // end class

}   // end namespace

#endif
