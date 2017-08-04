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

#include "ObjModelTriangleMeshParser.h"
#include "ObjModelNormalCalculator.h"
using RFeatures::ObjModelTriangleMeshParser;
using RFeatures::ObjModelBoundaryParser;
using RFeatures::ObjModelTriangleParser;
using RFeatures::ObjModel;
using RFeatures::ObjPoly;
#include <boost/foreach.hpp>
#include <cassert>
#include <algorithm>
#include <stack>

// public
ObjModelTriangleMeshParser::ObjModelTriangleMeshParser( const ObjModel::Ptr m) : _model(m), _bparser(NULL)
{
}   // end ctor


// public
void ObjModelTriangleMeshParser::reset()
{
    _bparser = NULL;
    _tparsers.clear();
    _tparsersSet.clear();
    _parsedFaces.clear();
}   // end reset


// public
void ObjModelTriangleMeshParser::setBoundaryParser( ObjModelBoundaryParser *bp)
{
    _bparser = bp;
}   // end setTriangleAcceptor


// public
bool ObjModelTriangleMeshParser::addTriangleParser( ObjModelTriangleParser *tp)
{
    bool added = false;
    if ( tp != NULL)
    {
        if ( !_tparsersSet.count(tp))
        {
            _tparsers.push_back(tp);
            _tparsersSet.insert(tp);
        }   // end if
        added = true;
    }   // end if
    return added;
}   // end addTriangleParser


struct ObjModelTriangleMeshParser::Triangle
{
    Triangle( int f, int r, int a) : nfid(-1), fid(f), vroot(r), va(a)
    {
        parser->_parsedFaces.insert(fid);
        vb = model->getFace(fid).getOpposite( vroot, va);
        parser->processTriangleParsers( fid, vroot, va, vb);
    }   // end ctor

    bool canTop() { return updateNext( vb, va);}    // Triangle on edge opposite root
    Triangle goTop() { return Triangle( nfid, vb, va);}     // va remains the same

    bool canLeft() { return updateNext( vroot, vb);}    // Triangle adjacent to edge root,b
    Triangle goLeft() { return Triangle( nfid, vroot, vb);} // vroot remains the same

    bool canRight() { return updateNext( va, vroot);}       // vroot and va swap
    Triangle goRight() { return Triangle( nfid, va, vroot);}

    static ObjModel::Ptr model;
    static ObjModelTriangleMeshParser *parser;
    static bool failed;

    int nfid, fid, vroot, va, vb;

private:
    bool updateNext( int r, int a)
    {
        nfid = -1;
        const IntSet& sfids = model->getSharedFaces( r, a);
        if ( sfids.size() > 2 || failed)
            failed = true;  // Not a valid triangulation
        else if ( parser->parseEdge( fid, r, a) && ( sfids.size() == 2))   // Must agree with parseEdge (which could be a buggy client)
        {
            nfid = *sfids.begin();
            if ( nfid == fid)
                nfid = *(++sfids.begin());
            if ( parser->_parsedFaces.count(nfid))
                nfid = -1; // If already discovered, there's no next face to get!
        }   // end else
        return nfid >= 0;
    }   // end updateNext
};  // end struct

// static definitions
ObjModel::Ptr ObjModelTriangleMeshParser::Triangle::model;
ObjModelTriangleMeshParser *ObjModelTriangleMeshParser::Triangle::parser(NULL);
bool ObjModelTriangleMeshParser::Triangle::failed(false);


// public
int ObjModelTriangleMeshParser::parse( int fid, const cv::Vec3d& planev)
{
    if ( fid < 0)
        fid = *_model->getFaceIds().begin();

    _parsedFaces.clear();
    Triangle::model = _model;
    Triangle::parser = this;
    Triangle::failed = false;
    std::stack<Triangle> *stack = new std::stack<Triangle>;

    const ObjPoly& face = _model->getFace(fid);
    int vroot = face.vindices[0];
    int va = face.vindices[1];

    // Decide parse ordering of face vertices?
    const double pvnorm = cv::norm(planev);
    if ( pvnorm > 0)
    {
        // Need to choose base vertices vroot and va to give normals that point
        const cv::Vec3d tstNorm = RFeatures::ObjModelNormalCalculator( _model)( vroot, va, face.vindices[2]);
        cv::Vec3d u;
        cv::normalize( planev, u);
        // If the face normal calculated is not in the direction of planev, swap starting vertices.
        if ( u.dot(tstNorm) < 0)
            std::swap( vroot, va);
    }   // end if

    Triangle t( fid, vroot, va);
    t.canLeft();
    t.canRight();
    stack->push( t);

    while ( !stack->empty())
    {
        t = stack->top();
        stack->pop();

        while ( !Triangle::failed)
        {
            if ( t.canTop())   // Try going top first
            {
                Triangle tnew = t.goTop();
                if ( t.canLeft() || t.canRight())
                    stack->push(t);
                t = tnew;
            }   // end if
            else if ( t.canLeft()) // Unable to go top, so try to go left...
            {
                Triangle tnew = t.goLeft();
                if ( t.canRight())
                    stack->push(t);
                t = tnew;
            }   // end else if
            else if ( t.canRight()) // Buggered again. Go right.
                t = t.goRight();
            else
                break;
        }   // end while
    }   // end while

    delete stack;
    informFinishedParsing();
    return (int)_parsedFaces.size();
}   // end parse



// private
void ObjModelTriangleMeshParser::processTriangleParsers( int fid, int vroot, int va, int vb)
{
    BOOST_FOREACH ( ObjModelTriangleParser* tp, _tparsers)
        tp->parseTriangle( fid, vroot, va, vb);
}   // end processTriangleParsers


// private
void ObjModelTriangleMeshParser::informFinishedParsing()
{
    BOOST_FOREACH ( ObjModelTriangleParser* tp, _tparsers)
        tp->finishedParsing();
}   // end informFinishedParsing


// private
bool ObjModelTriangleMeshParser::parseEdge( int fid, int r, int a)
{
    bool v = true;
    if ( _bparser)
        v = _bparser->parseEdge( fid, r, a);
    return v;
}   // end parseEdge
