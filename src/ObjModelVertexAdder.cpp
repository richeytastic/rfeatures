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

#include <ObjModelVertexAdder.h>
#include <ObjModelPolygonAreas.h>
#include <boost/heap/fibonacci_heap.hpp>
#include <cassert>
#include <queue>
using RFeatures::ObjModelVertexAdder;
using RFeatures::ObjModel;
using std::unordered_map;
using std::unordered_set;

namespace {

void checkAddLargeTriangles( std::queue<int>& fids, const IntSet& fset, const ObjModel* model, double maxTriangleArea)
{
    for ( int fid : fset)
    {
        if ( RFeatures::ObjModelPolygonAreas::calcFaceArea( model, fid) > maxTriangleArea)
            fids.push(fid);
    }   // end foreach
}   // end checkAddLargeTriangles

}   // end namespace


// public
ObjModelVertexAdder::ObjModelVertexAdder( ObjModel::Ptr model) : _model(model)
{
}   // end ctor
                                                        

// public
int ObjModelVertexAdder::addVerticesToMaxTriangleArea( double maxTriangleArea)
{
    std::queue<int> fids;
    checkAddLargeTriangles( fids, _model->faceIds(), _model.get(), maxTriangleArea);

    static const float ONE_THIRD = 1.0f/3;
    int nadded = 0;
    while ( !fids.empty())
    {
        const int fid = fids.front();
        fids.pop();

        // Subdivision position as mean of vertices
        const int* vidxs = _model->fvidxs(fid);
        const cv::Vec3f npos = (_model->vtx(vidxs[0]) + _model->vtx(vidxs[1]) + _model->vtx(vidxs[2])) * ONE_THIRD;
        int nvidx = _model->subDivideFace( fid, npos);

        // Check the newly created subdivided faces to see if they need to be subdivided also...
        checkAddLargeTriangles( fids, _model->faceIds(nvidx), _model.get(), maxTriangleArea);
        nadded++;
    }   // end while

    return nadded;
}   // end addVerticesToMaxTriangleArea


// public
int ObjModelVertexAdder::subdivideAndMerge( double maxTriangleArea)
{
    static const float ONE_THIRD = 1.0f/3;
    int nvadded = 0;

    // Find the set of faces with area greater than maxTriangleArea
    IntSet* fset = new IntSet;
    for ( int fid : _model->faceIds())
    {
        if ( RFeatures::ObjModelPolygonAreas::calcFaceArea( _model.get(), fid) > maxTriangleArea)
            fset->insert(fid);
    }   // end foreach

    // fedges will be the set of all edges that may be "edge-flipped"
    unordered_set<RFeatures::Edge, RFeatures::HashEdge> *fedges = new unordered_set<RFeatures::Edge, RFeatures::HashEdge>;
    while ( !fset->empty()) // While no more faces larger than the maximum triangle area...
    {
        for ( int fid : *fset)
        {
            // Get the set of edges that may be flipped from this triangle
            // (includes edges not sharing a single pair of texture coordinates).
            const int* vidxs = _model->fvidxs(fid);
            assert(vidxs);

            if ( _model->nspolys( vidxs[0], vidxs[1]) == 2)
                fedges->insert( RFeatures::Edge( vidxs[0], vidxs[1]));
            if ( _model->nspolys( vidxs[1], vidxs[2]) == 2)
                fedges->insert( RFeatures::Edge( vidxs[1], vidxs[2]));
            if ( _model->nspolys( vidxs[2], vidxs[0]) == 2)
                fedges->insert( RFeatures::Edge( vidxs[2], vidxs[0]));

            // Subdivide this face into three new polygons with new vertex at the centre.
            const cv::Vec3f npos = (_model->vtx(vidxs[0]) + _model->vtx(vidxs[1]) + _model->vtx(vidxs[2])) * ONE_THIRD;
            _model->subDivideFace( fid, npos);
            assert( _model->faceIds().count(fid) == 0);
            nvadded++;   // New vertex added
        }   // end foreach
        fset->clear();

        for ( const RFeatures::Edge& edge : *fedges)
        {
            const int v0 = edge[0];
            const int v1 = edge[1];
            const IntSet& sfids = _model->spolys( v0, v1);
            const int f0 = *sfids.begin();
            const int f1 = *(++sfids.begin());

            // If 1-to-1 mapping of geometry edge to texture edge, then just flip the face pair
            // normally. However, if there's a larger number of texture edges mapped to the
            // geometry edge, need to introduce a new vertex which adds new polygons.
            if ( _model->numTextureEdges( v0, v1) <= 1)
            {
                // Flip edge join for polygon pairs where the existing edge join is along the longer pair of opposing vertices.
                int v2 = _model->poly(f0).opposite( v0, v1);
                int v3 = _model->poly(f1).opposite( v0, v1);
                if ( cv::norm( _model->vtx(v2) - _model->vtx(v3)) < cv::norm( _model->vtx(v0) - _model->vtx(v1)))
                    _model->flipFacePair( v0, v1);

                assert( _model->faceIds().count(f0) > 0);
                assert( _model->faceIds().count(f1) > 0);

                // Note that the face indices don't change after flipping (though the areas of the faces may now be different).
                if ( fset->count(f0) == 0 && RFeatures::ObjModelPolygonAreas::calcFaceArea( _model.get(), f0) > maxTriangleArea)
                    fset->insert( f0);
                if ( fset->count(f1) == 0 && RFeatures::ObjModelPolygonAreas::calcFaceArea( _model.get(), f1) > maxTriangleArea)
                    fset->insert( f1);
            }   // end if
            else
            {
                // Since edge v0-->v1 is being subdivided, the old polygons on this edge will be removed
                // from the model so they also need to be removed from fset.
                fset->erase(f0);
                fset->erase(f1);

                const cv::Vec3f npos = (_model->vtx(v0) + _model->vtx(v1)) * 0.5f;  // New vertex midway between the edge vertices.
                const int nvidx = _model->addVertex( npos);
                _model->subDivideEdge( v0, v1, nvidx);
                nvadded++;
                for ( int nfid : _model->faceIds(nvidx))
                {
                    assert( _model->faceIds().count(nfid) > 0);
                    if ( fset->count(nfid) == 0 && RFeatures::ObjModelPolygonAreas::calcFaceArea( _model.get(), nfid) > maxTriangleArea)
                        fset->insert( nfid);
                }   // end for
            }   // end else
        }   // end foreach
        fedges->clear();
    }   // end while

    delete fset;
    delete fedges;

    return nvadded;
}   // end subdivideAndMerge
