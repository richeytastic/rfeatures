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

#include <VertexBoundaries.h>
using RFeatures::VertexBoundaries;
#include <boost/foreach.hpp>
#include <algorithm>
#include <cassert>


namespace {
struct ListSorterMinToMax {
    bool operator()( const std::list<int>* p0, const std::list<int>* p1) { return p0->size() < p1->size(); }
};  // end struct

struct ListSorterMaxToMin {
    bool operator()( const std::list<int>* p0, const std::list<int>* p1) { return p0->size() >= p1->size(); }
};  // end struct
}   // end namespace


// public
VertexBoundaries::~VertexBoundaries()
{
    const int n = (int)getNumBoundaries();
    for ( int i = 0; i < n; ++i)
        delete _blists[i];
    typedef std::pair<int, std::list<int>*> LPair;
    BOOST_FOREACH ( const LPair& lp, _subfirst)   // Incomplete sub-boundaries not yet in _blists
        delete lp.second;
}   // end dtor


// public
size_t VertexBoundaries::getNumBoundaries() const { return _blists.size();}

// public
const std::list<int>& VertexBoundaries::getBoundary( int i) const { return *_blists.at(i);}


// public
void VertexBoundaries::setEdge( int r, int a)
{
    if ( _subfirst.count(a))
    {
        _subfirst.at(a)->push_front(r);
        _subfirst[r] = _subfirst.at(a);   // New "first" point
        _subfirst.erase(a);
        r = checkForSubBoundarySplicing( r);
    }   // end if
    else if ( _sublast.count(r))
    {
        _sublast.at(r)->push_back(a);
        _sublast[a] = _sublast.at(r);   // New "last" point
        _sublast.erase(r);
        r = checkForSubBoundarySplicing( a);
    }   // end else if
    else if ( !_sublast.count(a) && !_subfirst.count(r))    // Ignore edge sections that we already have
    {   // Start to create a new sub-boundary
        std::list<int>* newBoundary = new std::list<int>;
        _subfirst[r] = _sublast[a] = newBoundary;
        newBoundary->push_back(r);
        newBoundary->push_back(a);
    }   // end else

    checkForCompletedBoundary( r);
}   // end setEdge


// public
void VertexBoundaries::sortBoundaries( bool maxFirst)
{
    if ( maxFirst)
        std::sort( _blists.begin(), _blists.end(), ListSorterMaxToMin());
    else
        std::sort( _blists.begin(), _blists.end(), ListSorterMinToMax());
}   // end sortBoundaries


// private
int VertexBoundaries::checkForCompletedBoundary( int i)
{
    if ( !_subfirst.count(i) || !_sublast.count(i))
        return -1;

    int cblen = 0;
    // Are the first and last indices of the same list equal, and there are more than three vertices?
    if ( (_subfirst.at(i) == _sublast.at(i)) && _subfirst.at(i)->size() > 3)
    {
        std::list<int> *completeBoundary = _subfirst.at(i);
        assert( completeBoundary->front() == completeBoundary->back());
        completeBoundary->pop_back();   // Remove the last element which is the same as the first
        _subfirst.erase(i);
        _sublast.erase(i);
        _blists.push_back( completeBoundary);
        cblen = (int)completeBoundary->size();   // Get the number of elements in the complete boundary
    }   // end if
    return cblen;
}   // end checkForCompletedBoundary


// private
int VertexBoundaries::checkForSubBoundarySplicing( int i)
{
    if ( !_subfirst.count(i) || !_sublast.count(i))
       return i;

    if ( _subfirst.at(i) != _sublast.at(i)) // Splicing together different sub-boundaries
    {
        std::list<int> *subboundary = _subfirst.at(i);
        assert( subboundary->front() == _sublast.at(i)->back());
        subboundary->pop_front();   // Remove before splicing so not duplicated
        _subfirst.erase(i);
        subboundary->splice( subboundary->begin(), *_sublast.at(i)); // O(1)
        delete _sublast.at(i);
        _sublast.erase(i);

        i = *subboundary->begin();
        _subfirst[i] = subboundary;
        _sublast[*subboundary->rbegin()] = subboundary;
    }   // end if
    return i;
}   // end checkForSubBoundarySplicing
