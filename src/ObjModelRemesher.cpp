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

#include <ObjModelRemesher.h>
#include <ObjModelVertexCrossingTimeCalculator.h>
#include <boost/foreach.hpp>
#include <cmath>
#include <cassert>
#include <iomanip>
#include <sstream>
#include <queue>
using RFeatures::ObjModelRemesher;
using RFeatures::ObjModelFastMarcher;
using RFeatures::ObjModel;
using RFeatures::ObjPoly;
using RFeatures::FaceAngles;

/*
// DEBUG - private
void ObjModelRemesher::printTime( int v, const boost::unordered_map<int,int>& srcs) const
{
    const int s = srcs.at(v);
    std::cerr << " (" << std::right << std::setw(2) << v << ")[" << std::right << std::setw(2) << s
              << "]:" << std::fixed << std::setprecision(2) << _vTimes.at(v).at(s) << "   ";
    //std::cerr << " (" << std::right << std::setw(2) << v << ")[" << std::right << std::setw(2) << s << "] ";
    //std::cerr << " [" << std::right << std::setw(2) << s << "] ";
}   // end printTime


// DEBUG - private
void ObjModelRemesher::printVertexTimes( const boost::unordered_map<int,int>& srcs) const
{
    const IntSet& vidxs = _inmod->getVertexIds();
    const int ndim = (int)sqrt((double)vidxs.size());
    std::vector<std::vector<int> > m(ndim);
    for ( int i = 0; i < ndim; ++i)
        m[i].resize(ndim);

    BOOST_FOREACH ( int vi, vidxs)
    {
        const cv::Vec3f& v = _inmod->getVertex(vi);
        m[(int)v[0]][(int)v[1]] = vi;
    }   // end foreach

    for ( int i = ndim-1; i >= 0; --i)
    {
        std::cerr << "    ";
        for ( int j = 0; j < ndim; ++j)
            printTime( m[j][i], srcs);
        std::cerr << std::endl;
    }   // end for
}   // end printVertexTimes


// DEBUG
size_t printFaces( const ObjModel::Ptr model)
{
    const IntSet& fids = model->getFaceIds();
    if ( fids.empty())
        return 0;
    std::cerr << "\t\t " << fids.size() << " faces" << std::endl;
    BOOST_FOREACH ( int fid, fids)
    {
        const ObjPoly& face = model->getFace( fid);
        std::cerr << "\t\t   F" << fid << ") " << face.vindices[0] << "," << face.vindices[1] << "," << face.vindices[2] << std::endl;
    }   // end foreach
    return fids.size();
}   // end printFaces


// DEBUG
void listSaddlePoints( const boost::unordered_map<int,IntSet>& xymap) // DEBUG
{
    int n = 0;
    typedef std::pair<int,IntSet> XY;
    BOOST_FOREACH ( const XY& xy, xymap)
    {
        const int x = xy.first;
        BOOST_FOREACH ( int y, xy.second)
        {
            std::cerr << "    [" << std::right << std::setw(2) << x << "]-->[" << std::right << std::setw(2) << y << "] " << std::endl;
            n++;
        }   // end foreach
    }   // end foreach
    std::cerr << "    " << n << " xymap" << std::endl;
}   // end listSaddlePoints
*/


ObjModel::Ptr createEmpty( const ObjModel::Ptr src)
{
    ObjModel::Ptr nobj = ObjModel::create();
    // Copy across the materials - but not the texture offsets!
    const int nmats = (int)src->getNumMaterials();
    for ( int m = 0; m < nmats; ++m)
    {
        const std::vector<cv::Mat>& ambient = src->getMaterialAmbient(m);
        const std::vector<cv::Mat>& diffuse = src->getMaterialDiffuse(m);
        const std::vector<cv::Mat>& specular = src->getMaterialSpecular(m);

        const int n = nobj->addMaterial();
        for ( int i = 0; i < (int)ambient.size(); ++i)
            nobj->addMaterialAmbient( n, ambient[i]);
        for ( int i = 0; i < (int)diffuse.size(); ++i)
            nobj->addMaterialDiffuse( n, diffuse[i]);
        for ( int i = 0; i < (int)specular.size(); ++i)
            nobj->addMaterialSpecular( n, specular[i]);
    }   // end for
    return nobj;
}   // end createEmpty


// private
void ObjModelRemesher::init()
{
    resetMaxDistanceHeap();
    _vTimes.clear();
    _nearestSources.clear();
    _saddlePoints.clear();
    _outmod = ObjModel::create( _inmod->getSpatialPrecision());
    const IntSet& vidxs = _inmod->getVertexIds();
    BOOST_FOREACH ( int A, vidxs)
        _nearestSources[A] = -1;   // Denote no source mapping initially
}   // end init


// private
void ObjModelRemesher::resetMaxDistanceHeap()
{
    while ( !_maxHeap.empty())
    {
        RFeatures::MaxVertex* v = _maxHeap.top();
        _maxHeap.pop();
        delete v;
    }   // end while
    _heapMap.clear();
}   // end resetMaxDistanceHeap


// private
int ObjModelRemesher::popMaxDistanceHeap( double& t)
{
    if ( _maxHeap.empty())
        return -1;
    RFeatures::MaxVertex* v = _maxHeap.top();
    _maxHeap.pop();
    const int vidx = v->vidx;
    t = v->time;
    _heapMap.erase( vidx);
    delete v;
    return vidx;
}   // end popMaxDistanceHeap


// private
void ObjModelRemesher::updateMaxHeap( int A, double t)
{
    if ( !_heapMap.count(A))
    {
        RFeatures::MaxVertex* v = _heapMap[A] = new RFeatures::MaxVertex( A, t);
        v->maxHeapHandle = _maxHeap.push(v);   // O(log(N))
    }   // end if
    else
    {
        RFeatures::MaxVertex* v = _heapMap.at(A);
        v->time = t;
        _maxHeap.decrease( v->maxHeapHandle);  // O(log(N))
    }   // end else
}   // end updateMaxHeap


// private
void ObjModelRemesher::updateNarrowBand( int A, double t)
{
    if ( !_inarrowBand.count(A))
    {
        RFeatures::MinVertex* v = _inarrowBand[A] = new RFeatures::MinVertex( A, t);
        v->minHeapHandle = _iminHeap.push(v);    // O(log(N))
    }   // end if
    else
    {
        RFeatures::MinVertex* v = _inarrowBand.at(A);
        v->time = t;
        _iminHeap.increase( v->minHeapHandle);  // O(log(N))
    }   // end else
}   // end updateNarrowBand


// public
ObjModelRemesher::ObjModelRemesher( const ObjModel::Ptr m, const ObjModelFastMarcher::SpeedFunctor *sf, FaceAngles *fa)
    : _inmod(m), _speedFunctor(sf), _faceAngles(fa), _delfa(false)
{
    if ( fa == NULL)
    {
        _faceAngles = new FaceAngles;
        _delfa = true;
    }   // end if
}   // end ctor


// public
ObjModelRemesher::~ObjModelRemesher()
{
    resetMaxDistanceHeap();
    if ( _delfa)
        delete _faceAngles;
}   // end dtor


// public
void ObjModelRemesher::createSaddleEdges( boost::unordered_map<int,IntSet>& xymap) const
{
    BOOST_FOREACH ( int vidx, _saddlePoints)
    {
        const int X = _nearestSources.at(vidx);
        const IntSet& cvs = _inmod->getConnectedVertices(vidx);
        BOOST_FOREACH ( int c, cvs)
        {
            const int Y = _nearestSources.at(c);
            if ( Y < X)
                xymap[X].insert(Y);
        }   // end foreach
    }   // end foreach
}   // end createSaddleEdges


// private
void ObjModelRemesher::setVertexNearestSource( int A, int ns)
{
    _nearestSources[A] = ns; // Record that this input vertex has been reached by the new source
    const double t = _vTimes.at(A).at(ns);
    _saddlePoints.erase(A);
    updateNarrowBand( A, t);
    updateMaxHeap( A, t);
}   // end setVertexNearestSource


// private
double ObjModelRemesher::calcTimeFromSource( int A, const cv::Vec3f& vS) const
{
    const cv::Vec3f& vA = _inmod->getVertex( A);
    return (*_speedFunctor)(A) * cv::norm( vA - vS); // Update crossing time at A
}   // end calcTimeFromSource


// public
int ObjModelRemesher::sample( int A, int npoints)
{
    return sample( A, npoints, false);
}   // end sample


// public
int ObjModelRemesher::sampleInterpolated( int A, int npoints)
{
    return sample( A, npoints, true);
}   // end sampleInterpolated


// private
int ObjModelRemesher::sample( int A, int npoints, bool interpolate)
{
    if ( npoints < 3 || npoints > _inmod->getNumVertices())
    {
        std::cerr << "[WARNING] RFeatures::ObjModelRemesher::remesh() : called with < 3 or > N"
                  << " points (where N is the number of unique vertices in the input model)." << std::endl;
        return -1;
    }   // end if

    // Get farthest point from A in the input model.
    ObjModelFastMarcher* fastMarcher = new ObjModelFastMarcher( _inmod, _speedFunctor, _faceAngles);
    A = fastMarcher->propagateFront( A);
    delete fastMarcher;

    init();

    RFeatures::ObjPolyInterpolator* interpolator = NULL;
    if ( interpolate)
        interpolator = new RFeatures::ObjPolyInterpolator( _inmod, _outmod, _nearestSources, _vTimes);

    int ns = 0;
    double oldt = DBL_MAX;
    while ( _outmod->getNumVertices() < npoints && A >= 0)
    {
        const cv::Vec3f vS = interpolate ? interpolator->interpolate( A) : _inmod->getVertex(A);
        // The newly iterpolated point should be closer to A. If it's not, it means that the
        // underlying (input) model has lower than required density in this region and that the
        // newly interpolated point should not be added as it cannot be referenced from any input vertex.
        const double newt = calcTimeFromSource( A, vS);
        if ( newt < oldt)
        {
            ns = _outmod->addVertex( vS);
            _vTimes[A][ns] = newt;
            setVertexNearestSource( A, ns);
        }   // end if

        _ifixedSet.clear();
        while ( !_inarrowBand.empty())
            expandInputFront( ns);

        const int oldA = A;
        do
        {
            A = popMaxDistanceHeap( oldt);
        } while ( oldA == A);
    }   // end while

    if ( interpolator)
        delete interpolator;

    return (int)_outmod->getNumVertices();
}   // end sample


// private
int ObjModelRemesher::popNarrowBand()
{
    RFeatures::MinVertex* v = _iminHeap.top();
    _iminHeap.pop();
    const int A = v->vidx;
    _ifixedSet.insert(A);
    _inarrowBand.erase(A);
    delete v;
    return A;
}   // end popNarrowBand


// private
void ObjModelRemesher::expandInputFront( int Y)
{
    const int A = popNarrowBand();  // A fixed as closer to _nearestSources[A] == Y than any other source
    RFeatures::ObjModelVertexCrossingTimeCalculator vtcalc( _inmod, _vTimes, Y, _faceAngles);

    const IntSet& cvs = _inmod->getConnectedVertices( A);
    BOOST_FOREACH ( int C, cvs)
    {
        const double t = vtcalc( C, (*_speedFunctor)(C));
        _vTimes[C][Y] = t;

        if ( _ifixedSet.count(C))  // Only update non-fixed
            continue;

        double tC = DBL_MAX;
        const int X = _nearestSources.at(C);      // Current closest source to C
        if ( X >= 0)
            tC = _vTimes.at(C).at(X);    // Current best crossing time at C is from X

        if ( t < tC) // Y is at least as near to C as X is to C - NB sometimes Y == X
            setVertexNearestSource( C, Y);  // Replace X with Y as nearest interpolated source vertex to C and set C in narrow band for expansion
        else
            _saddlePoints.insert(A);    // Create a saddle record at input vertex A
    }   // end foreach
}   // end expandInputFront
