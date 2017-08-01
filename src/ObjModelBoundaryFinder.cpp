#include "ObjModelBoundaryFinder.h"
using RFeatures::ObjModelBoundaryFinder;
using RFeatures::ObjModel;
#include <boost/unordered_map.hpp>
#include <boost/foreach.hpp>
#include <algorithm>
#include <cassert>

struct ListSorterMinToMax {
    bool operator()( const std::list<int>* p0, const std::list<int>* p1) { return p0->size() < p1->size(); }
};  // end struct

struct ListSorterMaxToMin {
    bool operator()( const std::list<int>* p0, const std::list<int>* p1) { return p0->size() >= p1->size(); }
};  // end struct


// private
class ObjModelBoundaryFinder::Boundaries
{
public:
    ~Boundaries()
    {
        const int n = (int)getNumBoundaries();
        for ( int i = 0; i < n; ++i)
            delete _blists[i];
        typedef std::pair<int, std::list<int>*> LPair;
        BOOST_FOREACH ( const LPair& lp, _subfirst)   // Incomplete sub-boundaries not yet in _blists
            delete lp.second;
    }   // end dtor

    size_t getNumBoundaries() const { return _blists.size();}
    const std::list<int>& getBoundary( int i) const { return *_blists.at(i);}

    void setEdge( int r, int a)
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

    void sortBoundaries( bool maxFirst)
    {
        if ( maxFirst)
            std::sort( _blists.begin(), _blists.end(), ListSorterMaxToMin());
        else
            std::sort( _blists.begin(), _blists.end(), ListSorterMinToMax());
    }   // end sortBoundaries

private:
    std::vector<std::list<int>*> _blists;                  // Holds completed boundaries
    boost::unordered_map<int, std::list<int>*> _subfirst;  // Boundaries indexed by "first" vertex
    boost::unordered_map<int, std::list<int>*> _sublast;   // Boundaries indexed by "last" vertex

    int checkForCompletedBoundary( int i)
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

    int checkForSubBoundarySplicing( int i)
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
};  // end class


// public
ObjModelBoundaryFinder::ObjModelBoundaryFinder( const ObjModel::Ptr m, const std::list<int>* bvts)
    : _model(m), _boundaries(NULL)
{
    reset( bvts);
}   // end ctor


// public
ObjModelBoundaryFinder::~ObjModelBoundaryFinder()
{
    delete _boundaries;
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
            assert( _model->getVertexIds().count(b));
            _bverts[b] = lastb;
            lastb = b;
        }   // end foreach
    }   // end if

    if ( _boundaries)
        delete _boundaries;
    _boundaries = new Boundaries;
}   // end reset


// public
size_t ObjModelBoundaryFinder::getNumBoundaries() const
{
    return _boundaries->getNumBoundaries();
}   // end getNumBoundaries


// public
void ObjModelBoundaryFinder::sortBoundaries( bool maxFirst)
{
    _boundaries->sortBoundaries( maxFirst);
}   // end sortBoundaries


// public
const std::list<int>& ObjModelBoundaryFinder::getBoundary( int i) const
{
    return _boundaries->getBoundary(i);
}   // end getBoundary


// protected
bool ObjModelBoundaryFinder::parseEdge( int fid, int v0, int v1)
{
    bool parse = true;
    if ( _model->getNumSharedFaces( v0, v1) == 1
            || (_bverts.count(v0) && _bverts.count(v1) && (_bverts.at(v0) == v1 || _bverts.at(v1) == v0)))
    {
        _boundaries->setEdge( v0, v1);
        parse = false;  // Stop parsing by the ObjModelTriangleMeshParser beyond this edge
    }   // end if
    return parse;
}   // end parseEdge
