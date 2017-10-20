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
#include <boost/foreach.hpp>
#include <algorithm>
#include <cassert>
#include <iostream>
using RFeatures::VertexBoundaries;
using RFeatures::Edge;


namespace {
struct ListSorterMinToMax {
    bool operator()( const std::list<int>* p0, const std::list<int>* p1) { return p0->size() < p1->size(); }
};  // end struct

struct ListSorterMaxToMin {
    bool operator()( const std::list<int>* p0, const std::list<int>* p1) { return p0->size() >= p1->size(); }
};  // end struct
}   // end namespace


// public
VertexBoundaries::VertexBoundaries() {}


// public
VertexBoundaries::~VertexBoundaries()
{
    const int n = (int)getNumBoundaries();
    for ( int i = 0; i < n; ++i)
        delete _blists[i];
    typedef std::pair<int, std::list<int>*> LPair;
    BOOST_FOREACH ( const LPair& lp, _front)   // Incomplete sub-boundaries not yet in _blists
        delete lp.second;
}   // end dtor


// public
size_t VertexBoundaries::getNumBoundaries() const
{
    return _blists.size();
}   // end getNumBoundaries


// public
const std::list<int>& VertexBoundaries::getBoundary( int i) const
{
    assert( i < (int)_blists.size() && i >= 0);
    return *_blists.at(i);
}   // end getBoundary


// public
void VertexBoundaries::sortBoundaries( bool maxFirst)
{
    if ( getNumBoundaries() <= 1)   // Nothing to sort
        return;

    if ( maxFirst)
        std::sort( _blists.begin(), _blists.end(), ListSorterMaxToMin());
    else
        std::sort( _blists.begin(), _blists.end(), ListSorterMinToMax());
}   // end sortBoundaries


// public
void VertexBoundaries::checkBoundaries( const ObjModel::Ptr model) const
{
    std::cerr << "[INFO] RFeatures::VertexBoundaries::checkBoundaries" << std::endl;
    std::cerr << " + " << _blists.size() << " boundaries" << std::endl;
    BOOST_FOREACH ( const std::list<int>* boundary, _blists)
    {
        const int fst = boundary->front();
        const int lst = boundary->back();
        std::cerr << "  " << fst << " --> " << lst << " : " << boundary->size() << " (first ";
        const bool conn = model->getConnectedVertices(fst).count(lst) > 0;  // First connected to last?
        if ( !conn)
            std::cerr << "not ";
        std::cerr << "connected to last)" << std::endl;
    }   // end foreach

    const int ntail = _front.size();
    const int nhead = _back.size();
    assert( ntail == nhead);
    std::cerr << " + " << ntail << " semi-boundaries" << std::endl;

    typedef std::pair<int, std::list<int>*> LPair;
    BOOST_FOREACH ( const LPair& lp, _front)
    {
        const std::list<int>* semi = lp.second;
        const int fst = semi->front();
        const int lst = semi->back();
        std::cerr << "  " << fst << " --> " << lst << " : " << semi->size() << " (first ";
        const bool conn = model->getConnectedVertices(fst).count(lst) > 0;  // First connected to last?
        if ( !conn)
            std::cerr << "not ";
        std::cerr << "connected to last)" << std::endl;
    }   // end foreach
}   // end checkBoundaries


// public
void VertexBoundaries::finish( const ObjModel::Ptr model)
{
    typedef std::pair<int, std::list<int>*> LPair;

    IntSet spliced;
    do
    {
        std::vector<int> rfronts;
        BOOST_FOREACH ( const LPair& lp, _front)
            rfronts.push_back( lp.first);

        // Check endpoint combinations
        spliced.clear();
        const int n = rfronts.size();
        for ( int i = 0; i < n; ++i)    // Start-points of semi-boundaries
        {
            const int f0 = rfronts[i];
            if ( spliced.count(f0) > 0)    // Skip if spliced
                continue;

            for ( int j = 1; j < n; ++j)
            {
                const int f1 = rfronts[j];
                if ( spliced.count(f1) > 0)    // Skip if spliced
                    continue;

                const std::list<int>* semi = _front.at(f1);
                if ( model->getConnectedVertices(f0).count(semi->front()) > 0)  // Connected on the model?
                {
                    if ( checkAndSplice( f0, semi->front()))
                    {
                        spliced.insert(f0);
                        spliced.insert(semi->front());
                    }   // end if
                }   // end if
                else if ( model->getConnectedVertices(f0).count(semi->back()) > 0)  // Connected on the model?
                {
                    if ( checkAndSplice( f0, semi->back()))
                    {
                        spliced.insert(f0);
                        spliced.insert(semi->back());
                    }   // end if
                }   // end if
            }   // end for
        }   // end for

        // Check if list endpoints are connected.
        std::vector<std::list<int>* > semis;    // Copy out since modifying _front and _back
        BOOST_FOREACH ( const LPair& lp, _front)
            semis.push_back( lp.second);
        BOOST_FOREACH ( const std::list<int>* semi, semis)
        {
            const int fst = semi->front();
            const int lst = semi->back();
            if ( model->getConnectedVertices(fst).count(lst) > 0)  // First connected to last?
                finishBoundary( fst, lst);
        }   // end forech
    } while ( !spliced.empty());

    checkBoundaries( model);
}   // end finish


// public
void VertexBoundaries::setEdge( int r, int a)
{
    assert( r != a);
    if ( checkAndSplice(r,a))
        return;

    RFeatures::Edge nedge(r,a);
    if ( _edgeSet.count(nedge) > 0)
    {
        std::cerr << "[WARNING] RFeatures::VertextBoundaries::setEdge: Trying to set duplicate edge "
                  << nedge.v0 << " --> " << nedge.v1 << std::endl;
        return;
    }   // end if
    _edgeSet.insert( nedge);

    // Check existing sub-boundary heads and tails
    if ( _front.count(a))
    {   // New tail will be 'r'
        _front.at(a)->push_front(r);
        _front[r] = _front.at(a);
        _front.erase(a);
    }   // end if
    else if ( _front.count(r))
    {   // New tail will be 'a'
        _front.at(r)->push_front(a);
        _front[a] = _front.at(r);
        _front.erase(r);
    }   // end else if
    else if ( _back.count(a))
    {   // New head will be 'r'
        _back.at(a)->push_back(r);
        _back[r] = _back.at(a);
        _back.erase(a);
    }   // end else if
    else if ( _back.count(r))
    {   // New head will be 'a'
        _back.at(r)->push_back(a);
        _back[a] = _back.at(r);
        _back.erase(r);
    }   // end else if
    else
    {   // Edge segment not connected to existing edge, so start new sub-boundary
        std::list<int>* newBoundary = new std::list<int>();
        newBoundary->push_back(r);
        newBoundary->push_back(a);
        _front[r] = _back[a] = newBoundary;
    }   // end else
}   // end setEdge


// private
void VertexBoundaries::finishBoundary( int x, int y)
{
    const std::list<int> *finb = ( _front.count(x) > 0) ?  _front.at(x) : _back.at(x);
    assert((finb->front() == x && finb->back() == y) || (finb->front() == y && finb->back() == x));
    _blists.push_back( finb);

    // Remove head and tail references to x and y and set them within the boundary.
    _front.erase(x);
    _front.erase(y);
    _back.erase(x);
    _back.erase(y);
}   // end finishBoundary


// private
bool VertexBoundaries::checkAndSplice( int x, int y)
{
    std::list<int> *sb0 = NULL;
    bool sb0F = false;
    bool sb0B = false;
    if ( _front.count(x) > 0)
    {
        sb0 = _front.at(x);
        sb0F = true;
    }   // end if
    else if ( _back.count(x) > 0)
    {
        sb0 = _back.at(x);
        sb0B = true;
    }   // end else if

    std::list<int> *sb1 = NULL;
    bool sb1F = false;
    bool sb1B = false;
    if ( _front.count(y) > 0)
    {
        sb1 = _front.at(y);
        sb1F = true;
    }   // end if
    else if ( _back.count(y) > 0)
    {
        sb1 = _back.at(y);
        sb1B = true;
    }   // end else if

    // To splice, both vertices x and y must be at the tail or head of an existing sub-boundary.
    if ( sb0 == NULL || sb1 == NULL)
       return false;

    assert( sb0->size() > 1);
    assert( sb1->size() > 1);

    if ( sb0 == sb1)    // Complete the boundary if same (endpoints should be x and y)
        finishBoundary(x,y);
    else
    {
        // Splice two separate semi-boundaries (sb0 = ABC, sb1 = DEF)
        if ( sb0B && sb1F)      // (ABC, DEF --> ABCDEF)
            sb0->splice( sb0->end(), *sb1, sb1->begin(), sb1->end());
        else if ( sb0F && sb1B) // (ABC, DEF --> DEFABC)
            sb0->splice( sb0->begin(), *sb1, sb1->begin(), sb1->end());
        else if ( sb0B && sb1B) // (ABC, DEF --> ABCFED)
        {
            sb1->reverse(); // Can't do reverse iterator range? Not sure why - list is doubly linked...
            sb0->splice( sb0->end(), *sb1, sb1->begin(), sb1->end());
        }   // end else if
        else if ( sb0F && sb1F) // (ABC, DEF --> FEDABC)
        {
            sb1->reverse();
            sb0->splice( sb0->begin(), *sb1, sb1->begin(), sb1->end());
        }   // end else if

        const int a = *sb0->begin();
        const int b = *sb0->rbegin();

        // Remove head and tail references to x and y and set them within the boundary.
        _front.erase(x);
        _front.erase(y);
        _back.erase(x);
        _back.erase(y);

        _front[a] = sb0;
        _back[b] = sb0;
        delete sb1;
    }   // end else

    return true;
}   // end checkAndSplice
