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

#include <ObjModelComponentFinder.h>
#include <ObjModelTriangleMeshParser.h>
#include <ObjModelMeshTraversalRecorder.h>
using RFeatures::ObjModelComponentFinder;
using RFeatures::ObjModelBoundaryFinder;
using RFeatures::ObjModelTriangleMeshParser;
using RFeatures::ObjModelMeshTraversalRecorder;
using RFeatures::ObjModel;
#include <algorithm>
#include <cassert>


// public
ObjModelComponentFinder::Ptr ObjModelComponentFinder::create( const ObjModelBoundaryFinder::Ptr bf)
{
    return Ptr( new ObjModelComponentFinder(bf), [](auto d){ delete d;});
}   // end create


// private
ObjModelComponentFinder::ObjModelComponentFinder( const ObjModelBoundaryFinder::Ptr bf) : _bf(bf)
{}   // end ctor


// private
ObjModelComponentFinder::~ObjModelComponentFinder()
{
    // Only delete if more than 1 component.
    if ( size() > 1)
        std::for_each( std::begin(_components), std::end(_components), [](auto s){delete s;});
    _components.clear();
    _cb.clear();
    _lb.clear();
}   // end dtor


// public
size_t ObjModelComponentFinder::findComponents()
{
    const int nbs = (int)_bf->size(); // Num boundaries on the model
    ObjModel::Ptr model = _bf->getObject();
    _components.clear();
    _cb.clear();
    _lb.clear();
    if ( nbs <= 1)
    {
        const IntSet* cset = &model->getFaceIds();
        _components.push_back( cset);
        if ( nbs == 1)
        {
            _cb[cset].insert(0);
            _lb[cset] = 0;
        }   // end if
        return 1;
    }   // end if

    ObjModelTriangleMeshParser parser(model);   // For parsing the model components
    ObjModelMeshTraversalRecorder vrecorder;    // Record the vertices parsed
    parser.addTriangleParser( &vrecorder);
    const IntSet& traversed = vrecorder.traversed();    // Reference to the traversed vertices

    for ( int i = 0; i < nbs; ++i)
    {
        const std::list<int>& blist = _bf->boundary(i); // Get boundary (list of ordered vertices)
        const int sfidx = *model->getFaceIds(*blist.begin()).begin();  // A polygon attached to first vertex in boundary.

        // If a vertex from this boundary is in set of already parsed vertices, record component it maps to.
        if ( traversed.count(blist.front()) > 0)
        {
            // Find which of the components already parsed that this vertex is in.
            const IntSet* rset = NULL;
            for ( const IntSet* fset : _components)
            {
                if ( fset->count(sfidx) > 0)
                {
                    rset = fset;
                    break;
                }   // end if
            }   // end for

            assert(rset);
            _cb[rset].insert(i);   // Map boundary i to this component
        }   // end if
        else
        {   // Otherwise, this is a boundary on a new component, and it must be the longest for the component.
            IntSet *cset = new IntSet;
            _components.push_back(cset);
            parser.setParseSet(cset);
            parser.parse( sfidx);     // Parse the model starting at a face ID attached to boundary i.
            _cb[cset].insert(i);      // Map boundary index to the just parsed component.
            _lb[cset] = i;          // Set as the longest boundary for the component.
        }   // end else
    }   // end for

    // Sort components in descending order of number of polygons.
    std::sort( std::begin(_components), std::end(_components), []( auto p0, auto p1){return p1->size() < p0->size();});
    return _components.size();
}   // end findComponents


// public
const IntSet* ObjModelComponentFinder::component( int i) const
{
    if ( i >= (int)_components.size() || i < 0)
        return NULL;
    return _components.at(i);
}   // end component


// public
int ObjModelComponentFinder::numComponentBoundaries( int i) const
{
    const IntSet* cb = cboundaries(i);
    if ( cb == NULL)
        return -1;
    return (int)cb->size();
}   // end numComponentBoundaries


// public
const IntSet* ObjModelComponentFinder::cboundaries( int i) const
{
    const IntSet* c = component(i);
    if ( c == NULL || _cb.count(c) == 0)
        return NULL;
    return &_cb.at(c);
}   // end cboundaries


// public
int ObjModelComponentFinder::lboundary( int i) const
{
    const IntSet* c = component(i);
    if ( c == NULL)         // No component c
        return -2;
    if ( _cb.count(c) == 0) // No boundaries stored on component c
        return -1;
    return _lb.at(c);
}   // end lboundary
