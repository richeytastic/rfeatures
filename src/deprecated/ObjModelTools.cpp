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

#include "ObjModelTools.h"
using RFeatures::ObjModel;
using RFeatures::ObjPoly;
using RFeatures::Edge;
using RFeatures::ObjComponentFinder;
using RFeatures::ObjExtentsFinder;
#include <cassert>
#include <iostream>


// public
ObjComponentFinder::ObjComponentFinder( const ObjModel::Ptr& m) : _om(m)
{}   // end ctor



// Recursively breadth-first expands the front of connected vertices for a component.
void findConnectedVertices( const ObjModel::Ptr& om,
                            std::vector<int>& srchFront,    // The vertices to check in this iteration
                            boost::unordered_set<int>& segmentSet,  // The whole segment so far
                            std::vector<int>& segmentVec)           // Segment as a vector
{
    if ( srchFront.empty())
        return;

    std::vector<int> srchAdded;

    // Look at all the vertices added to the vertex front
    BOOST_FOREACH ( const int& uvid, srchFront)
    {
        const boost::unordered_set<int>& conn = om->getConnectedUniqueVertices(uvid);

        BOOST_FOREACH ( const int& vid, conn)
        {
            if ( segmentSet.count(vid))
                continue;

            srchAdded.push_back(vid);
            segmentSet.insert(vid);
            segmentVec.push_back(vid);
        }   // end foreach
    }   // end foreach

    // Recurse, if more vertices were added to the front
    findConnectedVertices( om, srchAdded, segmentSet, segmentVec);
}   // end findConnectedVertices


// public
ObjModel::Ptr ObjComponentFinder::getConnectedComponent( int uvidx) const
{
    boost::unordered_set<int>* segmentSet = new boost::unordered_set<int>;
    std::vector<int>* srchFront = new std::vector<int>;
    std::vector<int>* segmentVec = new std::vector<int>;
    srchFront->push_back(uvidx);
    segmentSet->insert(uvidx);
    segmentVec->push_back(uvidx);
    findConnectedVertices( _om, *srchFront, *segmentSet, *segmentVec);
    ObjModel::Ptr nom = _om->createFromUniqueVertices( *segmentVec);
    delete segmentSet;
    delete srchFront;
    delete segmentVec;
    return nom;
}   // end getConnectedComponent


// public
ObjModel::Ptr RFeatures::segmentSurface( const ObjModel::Ptr om, const std::vector<int>& bids, int inId)
{
    boost::unordered_set<int>* segmentSet = new boost::unordered_set<int>( bids.begin(), bids.end()); // Define the boundary
    std::vector<int>* srchFront = new std::vector<int>;
    std::vector<int>* segmentVec = new std::vector<int>;
    srchFront->push_back(inId);
    segmentSet->insert(inId);
    segmentVec->push_back(inId);
    findConnectedVertices( om, *srchFront, *segmentSet, *segmentVec); // Recursively check (and populate) srchFront
    ObjModel::Ptr nom = om->createFromUniqueVertices( *segmentVec);
    delete segmentSet;
    delete srchFront;
    delete segmentVec;
    return nom;
}   // end segmentSurface



// public
ObjModel::Ptr RFeatures::segmentNotSurface( const ObjModel::Ptr om, const std::vector<int>& bids, int inId)
{
    boost::unordered_set<int>* segmentSet = new boost::unordered_set<int>( bids.begin(), bids.end()); // Define the boundary
    std::vector<int>* srchFront = new std::vector<int>;
    std::vector<int>* segmentVec = new std::vector<int>;
    srchFront->push_back(inId);
    segmentSet->insert(inId);
    segmentVec->push_back(inId);
    findConnectedVertices( om, *srchFront, *segmentSet, *segmentVec); // segment now contains the vertices we DON'T want
    std::vector<int>* cverts = new std::vector<int>;    // Will contain the vertices we DO want
    const IntSet& uvidxs = om->getUniqueVertexIds();
    BOOST_FOREACH ( const int& uvidx, uvidxs)
    {
        if (!segmentSet->count(uvidx))  // If this unique vertex ISN'T in the grown region, add it to the set we want
            cverts->push_back(uvidx);
    }   // end for
    ObjModel::Ptr nom = om->createFromUniqueVertices( *cverts);
    delete segmentSet;
    delete srchFront;
    delete segmentVec;
    delete cverts;
    return nom;
}   // end segmentNotSurface


// public
bool RFeatures::areEndPointsConnected( const ObjModel::Ptr om, const std::vector<int>& uvtxs)
{
    if ( uvtxs.empty())
    {
        std::cerr << "WARNING RFeatures::areEndPointsConnected: uvtxs is empty!" << std::endl;
        return false;
    }   // end if

    if ( uvtxs.size() == 1)
    {
        std::cerr << "WARNING RFeatures::areEndPointsConnected: uvtxs contains only a single element!" << std::endl;
        return true;
    }   // end if

    const boost::unordered_set<int>& cuvs = om->getConnectedUniqueVertices( *uvtxs.begin());
    return cuvs.count( *uvtxs.rbegin()) > 0;
}   // end areEndPointsConnected


// public
double RFeatures::calcMeanEdgeDistance( const ObjModel::Ptr om)
{
    const IntSet& edgeIds = om->getEdgeIds();
    double totDist = 0;
    BOOST_FOREACH ( const int& eid, edgeIds)
    {
        const Edge& e = om->getEdge(eid);
        totDist += cv::norm( om->getUniqueVertex(e.v0) - om->getUniqueVertex(e.v1));
    }   // end foreach
    return totDist / edgeIds.size();
}   // end calcMeanEdgeDistance


// public
double RFeatures::calcMeanEdgeDistance( const ObjModel::Ptr om, std::vector<double>& elens)
{
    const IntSet& edgeIds = om->getEdgeIds();
    elens.resize(edgeIds.size());
    double totDist = 0;
    int i = 0;
    BOOST_FOREACH ( const int& eid, edgeIds)
    {
        const Edge& e = om->getEdge(eid);
        const double elen = cv::norm( om->getUniqueVertex(e.v0) - om->getUniqueVertex(e.v1));
        elens[i++] = elen;
        totDist += elen;
    }   // end foreach
    return totDist / edgeIds.size();
}   // end calcMeanEdgeDistance


// public
ObjExtentsFinder::ObjExtentsFinder( const ObjModel::Ptr& om)
{
    const IntSet& uvidxs = om->getUniqueVertexIds();
    const int n = (int)uvidxs.size();
    cv::Vec3d vs(0,0,0);
    _minExtent = cv::Vec3f( FLT_MAX, FLT_MAX, FLT_MAX);
    _maxExtent = -_minExtent;

    BOOST_FOREACH ( const int& uvidx, uvidxs)
    {
        const cv::Vec3f& uv = om->getUniqueVertex( uvidx);
        vs[0] += uv[0];
        vs[1] += uv[1];
        vs[2] += uv[2];

        _minExtent[0] = std::min<float>( _minExtent[0], uv[0]);
        _minExtent[1] = std::min<float>( _minExtent[1], uv[1]);
        _minExtent[2] = std::min<float>( _minExtent[2], uv[2]);

        _maxExtent[0] = std::max<float>( _maxExtent[0], uv[0]);
        _maxExtent[1] = std::max<float>( _maxExtent[1], uv[1]);
        _maxExtent[2] = std::max<float>( _maxExtent[2], uv[2]);
    }   // end foreach
    _mean = cv::Vec3f( float(vs[0]/n), float(vs[1]/n), float(vs[2]/n));
}   // end ctor


