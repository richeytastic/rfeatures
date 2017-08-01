#include <ObjModelFastMarcher.h>
#include <ObjModelVertexCrossingTimeCalculator.h>
using RFeatures::ObjModelFastMarcher;
using RFeatures::ObjModel;
using RFeatures::MinVertex;
using RFeatures::MaxVertex;
#include <boost/foreach.hpp>
#include <cfloat>
#include <cmath>


// Returns true if v0 is considered to be less than v1 for a max-heap (v0 goes below v1)
bool RFeatures::VertexMinTimeComparator::operator()( const MinVertex* v0, const MinVertex* v1) const
{
    return v0->time >= v1->time;  // Reversed because boost creates a max-heap
}   // end operator()


// Returns true if v1 is considered to be less than v0 for a max-heap (v0 goes above v1)
bool RFeatures::VertexMaxTimeComparator::operator()( const MaxVertex* v0, const MaxVertex* v1) const
{
    return v0->time <= v1->time;  // Reversed because boost creates a max-heap
}   // end operator()


ObjModelFastMarcher::ObjModelFastMarcher( const ObjModel::Ptr& m,
                                          const SpeedFunctor* sf,
                                          const RFeatures::FaceAngles* fa)
    : _model(m), _speedFunctor(sf), _faceAngles(fa)
{
    reset();
}   // end ctor


// public
void ObjModelFastMarcher::reset()
{
    _time.clear();
    // Initialise the vertex crossing times to max
    const IntSet& vidxs = _model->getVertexIds();
    BOOST_FOREACH ( int vid, vidxs)
        _time[vid] = DBL_MAX;
}   // end reset


// DEBUG
#include <iomanip>
void printTimes( int source, const boost::unordered_map<int, double>& times)
{
    std::cerr << "SOURCE: UV_" << source << std::endl;
    typedef std::pair<int, double> UTime;
    BOOST_FOREACH ( const UTime& ut, times)
        std::cerr << "UV_" << ut.first << ": " << std::right << std::fixed << std::setw(3) << ut.second << std::endl;
}   // end printTimes


// public
int ObjModelFastMarcher::propagateFront( int vidx, double t)
{
    assert( _model->getVertexIds().count(vidx) > 0);

    _narrowBand = new boost::unordered_map<int, MinVertex*>;
    _fixed = new boost::unordered_set<int>;
    _minHeap = new MinHeap;

    addVertex( vidx, t);
    int lastReached = vidx;
    while ( !_narrowBand->empty())
        lastReached = expandFront();

    //printTimes( vidx, _time);
    delete _narrowBand;
    delete _fixed;
    delete _minHeap;

    return lastReached;
}   // end propagateFront


// protected
int ObjModelFastMarcher::expandFront()
{
    const int vidx = popVertex();
    const IntSet& cvs = _model->getConnectedVertices( vidx);
    BOOST_FOREACH ( int cv, cvs)
    {
        if ( isFixed(cv))  // Ignore already fixed
            continue;

        const double F = getInverseSpeedAt(cv);  // Speed of progression at the point being updated
        RFeatures::ObjModelVertexCrossingTimeCalculator calc( _model, _time, _faceAngles);
        const double t = calc( cv, F);

        if ( t < _time.at(cv))
        {
            if ( !withinNarrowBand(cv))
                addVertex( cv, t);
            else
                updateVertex( cv, t);  // Update with faster crossing time
        }   // end if
    }   // end foreach
    return vidx;
}   // end expandFront


// protected
bool ObjModelFastMarcher::withinNarrowBand( int C) const
{
    return _narrowBand->count(C) > 0;
}   // end withinNarrowBand


// protected
bool ObjModelFastMarcher::isFixed( int C) const
{
    return _fixed->count(C) > 0;
}   // end isFixed


// protected
double ObjModelFastMarcher::getInverseSpeedAt( int C) const
{
    return (*_speedFunctor)(C);
}   // end getInverseSpeedAt


// protected
int ObjModelFastMarcher::popVertex()
{
    MinVertex* v = _minHeap->top();  // Reached "first" by current front
    _minHeap->pop();
    const int vidx = v->vidx;
    _fixed->insert( vidx);  // Fix this vertex
    _narrowBand->erase( vidx);
    delete v;
    return vidx;
}   // end popVertex


// protected
void ObjModelFastMarcher::addVertex( int vi, double t)
{
    MinVertex* v = (*_narrowBand)[vi] = new MinVertex( vi, t);
    _time[vi] = t;
    v->minHeapHandle = _minHeap->push( v); // O(log(N))
}   // end addVertex


// protected
void ObjModelFastMarcher::updateVertex( int vi, double t)
{
    MinVertex* v = _narrowBand->at(vi);
    _time[vi] = v->time = t;
    _minHeap->increase( v->minHeapHandle);   // O(log(N))
}   // end updateVertex
