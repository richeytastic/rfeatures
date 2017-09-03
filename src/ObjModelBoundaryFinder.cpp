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
    reset( bvts);
}   // end ctor


// public
ObjModelBoundaryFinder::~ObjModelBoundaryFinder()
{
    delete _vboundaries;
}   // end reset


// public
void ObjModelBoundaryFinder::reset( const std::list<int>* bvts)
{
    _bverts.clear();
    if ( bvts)
    {
        int lastb = *bvts->rbegin();
        BOOST_FOREACH ( const int& b, *bvts)
        {
            assert( model->getVertexIds().count(b));
            _bverts[b] = lastb;
            lastb = b;
        }   // end foreach
    }   // end if

    if ( _vboundaries)
        delete _vboundaries;
    _vboundaries = new RFeatures::VertexBoundaries;
}   // end reset


// public
size_t ObjModelBoundaryFinder::getNumBoundaries() const
{
    return _vboundaries->getNumBoundaries();
}   // end getNumBoundaries


// public
void ObjModelBoundaryFinder::sortBoundaries( bool maxFirst)
{
    _vboundaries->sortBoundaries( maxFirst);
}   // end sortBoundaries


// public
const std::list<int>& ObjModelBoundaryFinder::getBoundary( int i) const
{
    return _vboundaries->getBoundary(i);
}   // end getBoundary


// protected
bool ObjModelBoundaryFinder::parseEdge( int fid, int v0, int v1)
{
    bool parse = true;
    if ( model->getNumSharedFaces( v0, v1) == 1
            || (_bverts.count(v0) && _bverts.count(v1) && (_bverts.at(v0) == v1 || _bverts.at(v1) == v0)))
    {
        _vboundaries->setEdge( v0, v1);
        parse = false;  // Stop parsing by the ObjModelTriangleMeshParser beyond this edge
    }   // end if
    return parse;
}   // end parseEdge
