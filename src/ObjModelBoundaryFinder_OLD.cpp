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
using RFeatures::ObjModelBoundaryFinder;
using RFeatures::ObjModel;
#include <boost/foreach.hpp>
#include <cassert>


// public
ObjModelBoundaryFinder::ObjModelBoundaryFinder( const std::list<int>* bvts)
    : _vboundaries(NULL)
{
    if ( bvts)
        _bvts = *bvts;    // Copy in
}   // end ctor


// public
ObjModelBoundaryFinder::~ObjModelBoundaryFinder()
{
    delete _vboundaries;
}   // end dtor


// public
void ObjModelBoundaryFinder::reset()
{
    assert(model != NULL);
    _bverts.clear();
    if ( !_bvts.empty())
    {
        int lastb = *_bvts.rbegin();
        BOOST_FOREACH ( int b, _bvts)
        {
            assert( model->getVertexIds().count(b));
            _bverts[b] = lastb;
            lastb = b;
        }   // end foreach
    }   // end if

    if ( _vboundaries)
        delete _vboundaries;
    _vboundaries = new RFeatures::VertexBoundaries;
    _edgeSet.clear();
}   // end reset


// public
size_t ObjModelBoundaryFinder::getNumBoundaries() const
{
    return _vboundaries->getNumBoundaries();
}   // end getNumBoundaries


// public
const std::list<int>& ObjModelBoundaryFinder::getBoundary( int i) const
{
    return _vboundaries->getBoundary(i);
}   // end getBoundary


// protected
bool ObjModelBoundaryFinder::parseEdge( int fid, int v0, int v1)
{
    bool parse = true;
    if ( model->getNumSharedFaces( v0, v1) == 1 ||  // Check if a natural boundary
       (_bverts.count(v0) && _bverts.count(v1) && (_bverts.at(v0) == v1 || _bverts.at(v1) == v0)))  // Check if on specified boundary
    {
        _vboundaries->setEdge( v0, v1);
        parse = false;  // Stop parsing by the ObjModelTriangleMeshParser beyond this edge
    }   // end if
    RFeatures::Edge nedge(v0,v1);
    _edgeSet.insert(nedge);
    return parse;
}   // end parseEdge


// protected
void ObjModelBoundaryFinder::finishedParsing()
{
    _vboundaries->finish( model);
    assert( _vboundaries->getNumBoundaries() > 0);
    _vboundaries->sortBoundaries(true); // max first
}   // end finishedParsing
