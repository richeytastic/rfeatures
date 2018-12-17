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

#include <ObjModelBoundaryFinder.h>
#include <algorithm>
using RFeatures::ObjModelBoundaryFinder;
using RFeatures::ObjModel;


// public
ObjModelBoundaryFinder::Ptr ObjModelBoundaryFinder::create( const ObjModel* m)
{
    return Ptr( new ObjModelBoundaryFinder(m), [](ObjModelBoundaryFinder* d){ delete d;});
}   // end create


// private
ObjModelBoundaryFinder::ObjModelBoundaryFinder( const ObjModel* m) : _model(m)
{}   // end ctor


// private
ObjModelBoundaryFinder::~ObjModelBoundaryFinder()
{
    for ( std::list<int>* d : _boundaries)
        delete d;
    _boundaries.clear();
}   // end dtor


// public
size_t ObjModelBoundaryFinder::findOrderedBoundaryVertices( const IntSet& inbvtxs)
{
    _boundaries.clear();

    // If there are no boundary vertices, the object has no boundary!
    if ( inbvtxs.empty())
        return 0;

    IntSet bvtxs = inbvtxs; // Copy in the set of boundary vertices for exhausting

    std::list<int>* blst = new std::list<int>; // At least one boundary since non-empty set of boundary vertices.
    IntSet bset;   // Track the vertex indices being added for the current boundary
    int vidx = *bvtxs.begin();
    while ( vidx >= 0)
    {
        const int b0 = vidx;
        vidx = -1;

        blst->push_back(b0);
        bset.insert(b0);
        bvtxs.erase(b0);

        // Because b0 is a boundary vertex (a flat one at that), it must have exactly two
        // connected vertices that share single faces. Since we're constructing a boundary,
        // one of these connected vertices will already be in bset (assuming we're part way
        // through). We choose the vertex that's not in bset to continue the boundary. At the
        // start, either of these vertices can be used. If both of the vertices are found to
        // be in bset, the boundary is finished!
        const IntSet& cvtxs = _model->getConnectedVertices(b0);
        for ( int b1 : cvtxs)
        {
            if (( _model->getNumSharedFaces( b0, b1) == 1) && !bset.count(b1))
            {
                vidx = b1; // Found the next vertex on this boundary.
                break;
            }   // end if
        }   // end for

        if ( vidx < 0)
        {
            _boundaries.push_back(blst);    // Boundary done
            blst = nullptr;
            bset.clear();

            if ( !bvtxs.empty()) // Start another boundary if boundary vertices left to examine.
            {
                blst = new std::list<int>;  // New boundary.
                vidx = *bvtxs.begin();      // Starting vertex.
            }   // end if
        }   // end if
    }   // end while

    if ( blst)
        delete blst;

    // Sort boundaries in descending order of vertex count.
    std::sort( std::begin(_boundaries), std::end(_boundaries), []( std::list<int>* p0, std::list<int>* p1){return p1->size() < p0->size();});
    return _boundaries.size();
}   // end findOrderedBoundaryVertices


namespace {
void findAllBoundaryVertices( const ObjModel* model, IntSet& bvtxs)
{
    for ( int vidx : model->getVertexIds())
    {
        // If there exists a connected vertex to vidx such that that edge
        // is shared by only a single polygon, then vidx must be on the boundary.
        for ( int cv : model->getConnectedVertices( vidx))
        {
            if ( model->getNumSharedFaces( vidx, cv) == 1)
            {
                bvtxs.insert(vidx);
                break;
            }   // end if
        }   // end foreach
    }   // end for
}   // end findAllBoundaryVertices
}   // end namespace


// public
size_t ObjModelBoundaryFinder::findOrderedBoundaryVertices()
{
    IntSet bvtxs;  // When all gone, no more boundary vertices to check.
    findAllBoundaryVertices( _model, bvtxs);
    return findOrderedBoundaryVertices( bvtxs);
}   // end findOrderedBoundaryVertices

