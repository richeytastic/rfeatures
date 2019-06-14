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

#ifndef RFEATURES_OBJ_MODEL_MANIFOLDS_H
#define RFEATURES_OBJ_MODEL_MANIFOLDS_H

#include "ObjModelManifoldBoundaries.h"

/**
 * Find separate manifolds of a model as the distinct subsets
 * of triangles forming 2D triangulated manifolds. Some polygons can
 * not be included in triangulated manifolds and these are returned via
 * nonManifoldPolygons (typically these are polygons with an edge
 * that is shared by two or more other polygons).
 */
namespace RFeatures {

class ObjModelManifolds;

class rFeatures_EXPORT ObjManifold
{
public:
    // Return the set of faces that define this manifold.
    const IntSet& polygons() const { return _polys;}

    // Vertices are created on demand from the polygons.
    const IntSet& vertices() const;

    // Returns the (unordered) set of boundary edge ids. All edges are shared with a single face.
    // The ObjModelManifoldBoundaries object (returned from boundaries) orders these edges into boundary lists.
    const IntSet& edges() const { return _edges;}

    // Return the boundary lists of this manifold comprised of the ordered edges (returned from edges).
    // Note that these boundary lists are never computed from the edges if this function is never called.
    const ObjModelManifoldBoundaries& boundaries() const;

    // Create and return a new object that contains only this manifold's geometry.
    // The returned model will have its own unique vertex and polygon IDs different
    // to the vertex and polygon IDs stored in this manifold.
    ObjModel::Ptr toObject() const;

private:
    const ObjModel* _model;
    IntSet _polys;
    IntSet _edges;
    mutable IntSet _verts;
    mutable ObjModelManifoldBoundaries _bnds;

    friend class ObjModelManifolds;
    ObjManifold( const ObjModel*);
    ObjManifold( const ObjManifold&);
    ObjManifold& operator=( const ObjManifold&);
};  // end class


class rFeatures_EXPORT ObjModelManifolds
{
public:
    using Ptr = std::shared_ptr<ObjModelManifolds>;
    static Ptr create( ObjModel::Ptr);

    const ObjModel* cmodel() const { return _model.get();}
    ObjModel::Ptr model() { return _model;}

    size_t count() const { return _manfs.size();}

    // Returns the id of the manifold for the given polygon or -1 if not found.
    int manifoldId( int fid) const;

    // Returns manifold at position i. Manifolds are stored in descending order of # polygons.
    // By default, returns the largest (and maybe only) model manifold. Returns null if index out of range.
    const ObjManifold* manifold( int i) const;

    // Create and return a new model having only the top n manifolds (largest in terms of polygon count).
    // The returned model will only contain connected vertices that are part of the top n manifolds.
    Ptr reduceManifolds( int n);

private:
    ObjModel::Ptr _model;
    std::vector<ObjManifold*> _manfs;
    std::unordered_map<int, int> _poly2manf;    // Polygon to manifold ID.

    void _findManifolds();
    explicit ObjModelManifolds( ObjModel::Ptr);
    virtual ~ObjModelManifolds();
    ObjModelManifolds( const ObjModelManifolds&) = delete;
    void operator=( const ObjModelManifolds&) = delete;
};  // end class

}   // end namespace

#endif
