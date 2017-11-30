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

#include <ObjModelBoundaryFinder2.h>
using RFeatures::ObjModelBoundaryFinder2;
using RFeatures::ObjModel;
#include <boost/foreach.hpp>
#include <algorithm>


namespace
{

void findAllBoundaryVertices( const ObjModel::Ptr model, IntSet& buverts)
{
    const IntSet& uvidxs = model->getVertexIds();
    BOOST_FOREACH ( int uvidx, uvidxs)
    {
        // If there exists a connected vertex to uvidx such that that edge
        // is shared by only a single polygon, then uvidx must be on the boundary.
        const IntSet& cuvtxs = model->getConnectedVertices( uvidx);
        bool onBoundary = false;
        BOOST_FOREACH ( int cuv, cuvtxs)
        {
            if ( model->getNumSharedFaces( uvidx, cuv) == 1)
            {
                onBoundary = true;
                break;
            }   // end if
        }   // end foreach

        if ( onBoundary)
            buverts.insert(uvidx);
    }   // end for
}   // end findAllBoundaryVertices


struct ListSorterMinToMax {
    bool operator()( const std::list<int>& p0, const std::list<int>& p1) { return p0.size() < p1.size(); }
};  // end struct

struct ListSorterMaxToMin {
    bool operator()( const std::list<int>& p0, const std::list<int>& p1) { return p1.size() < p0.size(); }
};  // end struct

}   // end namespace


ObjModelBoundaryFinder2::ObjModelBoundaryFinder2( const ObjModel::Ptr model) : _model(model)
{}   // end ctor



// public
void ObjModelBoundaryFinder2::sortBoundaries( bool maxFirst)
{
    if ( maxFirst)
        std::sort( _boundaries.begin(), _boundaries.end(), ListSorterMaxToMin());
    else
        std::sort( _boundaries.begin(), _boundaries.end(), ListSorterMinToMax());
}   // end sortBoundaries


// public
size_t ObjModelBoundaryFinder2::findOrderedBoundaryVertices()
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
        const IntSet& cvtxs = _model->getConnectedVertices(b0);
        BOOST_FOREACH ( int b1, cvtxs)
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

    return _boundaries.size();
}   // end findOrderedBoundaryVertices


