#include "ObjModelBoundaryFinder.h"
using RFeatures::ObjModel;
using RFeatures::ObjModelBoundaryFinder;
#include <iostream>
#include <boost/foreach.hpp>


ObjModelBoundaryFinder::ObjModelBoundaryFinder( const ObjModel::Ptr& om) : _model(om)
{}   // end ctor


void findAllBoundaryVertices( const ObjModel::Ptr om, IntSet& buverts)
{
    const IntSet& uvidxs = om->getUniqueVertexIds();
    BOOST_FOREACH ( const int& uvidx, uvidxs)
    {
        // If there exists a connected vertex to uvidx such that that edge
        // is shared by only a single polygon, then uvidx must be on the boundary.
        const IntSet& cuvtxs = om->getConnectedUniqueVertices( uvidx);
        bool onBoundary = false;
        BOOST_FOREACH ( const int& cuv, cuvtxs)
        {
            if ( om->getNumSharedFaces( uvidx, cuv) == 1)
            {
                onBoundary = true;
                break;
            }   // end if
        }   // end foreach

        if ( onBoundary)
            buverts.insert(uvidx);
    }   // end for
}   // end findAllBoundaryVertices


// public
int ObjModelBoundaryFinder::findOrderedBoundaryUniqueVertexIndices()
{
    IntSet buverts;  // When all gone, no more boundary vertices to check.
    findAllBoundaryVertices( _model, buverts);

    // If there are no boundary vertices, the object has no boundary!
    if ( buverts.empty())
        return 0;

    _boundaries.resize(1);  // Assume only a single boundary initially
    std::list<int>* buvtxs = &_boundaries[0];
    IntSet bset;   // Track the vertex indices being added for the current boundary

    int uvidx = *buverts.begin();
    while ( uvidx >= 0)
    {
        const int b0 = uvidx;
        uvidx = -1;

        buvtxs->push_back(b0);
        bset.insert(b0);
        buverts.erase(b0);

        // Because b0 is a boundary vertex (a flat one at that), it must have exactly two
        // connected vertices that share single faces. Since we're constructing a boundary,
        // one of these connected vertices will already be in bset (assuming we're part way
        // through). We choose the vertex that's not in bset to continue the boundary. At the
        // start, either of these vertices can be used. If both of the vertices are found to
        // be in bset, the boundary is finished!
        const IntSet& cvtxs = _model->getConnectedUniqueVertices(b0);

        BOOST_FOREACH ( const int& b1, cvtxs)
        {
            if (( _model->getNumSharedFaces( b0, b1) == 1) && !bset.count(b1))
            {
                uvidx = b1; // Found the next vertex on this boundary.
                break;
            }   // end if
        }   // end foreach

        // Reset for next separate boundary on the model because
        // there are still more boundary vertices left to account for.
        if ( uvidx < 0 && !buverts.empty())
        {
            _boundaries.resize( _boundaries.size()+1);
            buvtxs = &(*_boundaries.rbegin());
            bset.clear();
            uvidx = *buverts.begin();
        }   // end if
    }   // end while

    return (int)_boundaries.size();
}   // end findOrderedBoundaryUniqueVertexIndices

