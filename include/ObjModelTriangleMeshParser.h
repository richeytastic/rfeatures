/************************************************************************
 * Copyright (C) 2019 Richard Palmer
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
 * Parses an object by traversing its edges using a consistent ordering of vertices so that adjacent
 * triangles *normally* have their "outward" faces defined in a similar direction (meaning that for
 * two triangles lying in the same plane, the inner product of their normal vectors is 1).
 * HOWEVER in cases where the triangulated mesh is twisted (e.g. like a mobius strip), this rule will
 * not hold and there will be two triangles lying in the same plane where the order of their vertices
 * results in surface normals being defined where their inner product is -1. If such a situation is
 * encountered, function twisted() will return true after the parse() function returns.
 * Parsing is started using the given polygon id (or the first one in the model if not given). Polygons
 * that can't be reached by parsing edges are not parsed. After all triangles that can be reached in
 * the parsing operation have been parsed, the set of parsed triangles is given by the set returned
 * from the parsed() function.
 */
#include "ObjModel.h"

namespace RFeatures {

// Parse the given triangle with a consistent vertex edge ordering.
// The vertices are always the vertex IDs in the associated model.
struct rFeatures_EXPORT ObjModelTriangleParser
{
    ObjModelTriangleParser() : model(nullptr) {}
    virtual ~ObjModelTriangleParser(){}
    virtual void parseTriangle( int fid, int vroot, int va, int vb) = 0;
    virtual void finishedParsing(){}   // Be informed when parsing all triangles finished.
    virtual void reset(){}  // Called in ObjModelTriangleMeshParser::setTriangleParser AFTER model set
    const ObjModel* model;
};  // end struct

// Given a triangle edge in the direction e[0]-->e[1], specify that parsing should progress beyond this
// edge by returning true from parseEdge(). Optionally, specify exactly which polygon parsing should
// go to next by setting out parameter pnfid. Returning false causes the mesh parser to treat this
// edge as a boundary (whether or not it is an actual boundary of the mesh). It's safe to always return
// true from this function in which case traversal will continue up to the actual bounds of the mesh.
// Parameter fid is the id of the triangle just parsed, and the vertices are the edge (either the top or
// the left edge of that face).
struct rFeatures_EXPORT ObjModelBoundaryParser
{
    ObjModelBoundaryParser() : model(nullptr) {}
    virtual ~ObjModelBoundaryParser(){}
    virtual bool parseEdge( int fid, const cv::Vec2i&, int& pnfid) = 0;
    virtual void finishedParsing(){}   // Be informed when parsing all triangles finished.
    virtual void reset(){}  // Called in ObjModelTriangleMeshParser::setBoundaryParser AFTER model set
    const ObjModel* model;
};  // end struct


class rFeatures_EXPORT ObjModelTriangleMeshParser
{
public:
    explicit ObjModelTriangleMeshParser( const ObjModel&);
    virtual ~ObjModelTriangleMeshParser();

    // Only triangles connected via a shared edge are included in the parsing.
    // Returns the number of triangles parsed.
    // startPoly: The ID of the model polygon to start parsing with.
    // planev:    The half of coordinate space to use as the "positive"
    //            half regarding the direction of the starting polygon's normal.
    //            This defines the consistent parse ordering of polygon vertices.
    //            If left as default (zero vector), an undefined ordering is used.
    int parse( int startPoly=0, const cv::Vec3d planev=cv::Vec3d(0,0,0), bool clearParsed=true);

    // If this function returns true after a call to parse, then the set of parsed
    // polygons involves an odd number of "twists" where edge vertices have their order
    // reversed and a consistent ordering of surface normals is not possible.
    bool twisted() const { return _twisted;}

    // Returns the set of polygons parsed from the last call to parse().
    // (the set of parsed polygons is cleared for every new call to parse()
    // by default, but may be held by setting clearParsed to false).
    const IntSet& parsed() const { return _parsed;}

    // Add processing delegates
    bool addTriangleParser( ObjModelTriangleParser*);   // Returns true if added or already set
    void setBoundaryParser( ObjModelBoundaryParser*);

    const ObjModel& model() const { return _model;}

private:
    const ObjModel& _model;
    bool _twisted;
    IntSet _parsed;
    ObjModelBoundaryParser* _bparser;
    std::unordered_set<ObjModelTriangleParser*> _tparsers;

    void processTriangleParsers( int, const cv::Vec3i&);
    void informFinishedParsing();
    bool parseEdge( int, const cv::Vec2i&, int&);
    struct Triangle;
};  // end class

}   // end namespace

#endif
