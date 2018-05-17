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

#include <ObjModelTriangleMeshParser.h>
#include <ObjModelNormals.h>
using RFeatures::ObjModelTriangleMeshParser;
using RFeatures::ObjModelBoundaryParser;
using RFeatures::ObjModelTriangleParser;
using RFeatures::ObjModel;
using RFeatures::ObjPoly;
#include <algorithm>
#include <cassert>
#include <stack>

// public
ObjModelTriangleMeshParser::ObjModelTriangleMeshParser( const ObjModel::Ptr m, IntSet *pfaces)
    : _model(m), _bparser(NULL), _parsedFaces(pfaces), _dodel(false)
{
    assert(m);
    if ( !_parsedFaces)
    {
        _parsedFaces = new IntSet;
        _dodel = true;
    }   // end if
}   // end ctorA


ObjModelTriangleMeshParser::~ObjModelTriangleMeshParser() { setParseSet(_parsedFaces);}


void ObjModelTriangleMeshParser::setParseSet( IntSet *pfaces)
{
    assert(pfaces);
    if ( _dodel)
    {
        delete _parsedFaces;
        _dodel = false;
    }   // end if
    _parsedFaces = pfaces;
}   // end setParseSet


// public
void ObjModelTriangleMeshParser::setBoundaryParser( ObjModelBoundaryParser* bp)
{
    _bparser = bp;
    assert(_bparser != NULL);
    _bparser->model = _model;
    _bparser->reset();
}   // end setBoundaryParser


// public
bool ObjModelTriangleMeshParser::addTriangleParser( ObjModelTriangleParser* tp)
{
    bool added = false;
    if ( tp != NULL)
    {
        if ( !_tparsers.count(tp))
        {
            tp->model = _model;
            tp->reset();
            _tparsers.insert(tp);
        }   // end if
        added = true;
    }   // end if
    return added;
}   // end addTriangleParser


struct ObjModelTriangleMeshParser::Triangle
{
    Triangle( ObjModelTriangleMeshParser* parser, ObjModel::Ptr m, int f, int r, int a, bool failed=false)
        : _parser(parser), _model(m), nfid(-1), fid(f), vroot(r), va(a), _failed(failed)
    {
        parser->_parsedFaces->insert(fid);
        vb = _model->getFace(fid).getOpposite( vroot, va);
        parser->processTriangleParsers( fid, vroot, va, vb);
    }   // end ctor

    bool canTop() { return updateNext( vb, va);}    // Triangle on edge opposite root
    Triangle goTop() { return Triangle( _parser, _model, nfid, vb, va, _failed);}     // va remains the same

    bool canLeft() { return updateNext( vroot, vb);}    // Triangle adjacent to edge root,b
    Triangle goLeft() { return Triangle( _parser, _model, nfid, vroot, vb, _failed);} // vroot remains the same

    bool canRight() { return updateNext( va, vroot);}       // vroot and va swap
    Triangle goRight() { return Triangle( _parser, _model, nfid, va, vroot, _failed);}

    bool failed() const { return _failed;}

private:
    bool updateNext( int r, int a)
    {
        nfid = -1;
        const IntSet& sfids = _model->getSharedFaces( r, a);
        if ( sfids.size() > 2 || _failed)
            _failed = true;  // Not a valid triangulation
        else if ( _parser->parseEdge( fid, r, a) && ( sfids.size() == 2))   // Must agree with parseEdge (which could be a buggy client)
        {
            nfid = *sfids.begin();
            if ( nfid == fid)
                nfid = *(++sfids.begin());
            if ( _parser->_parsedFaces->count(nfid))
                nfid = -1; // If already discovered, there's no next face to get!
        }   // end else
        return nfid >= 0;
    }   // end updateNext

    ObjModelTriangleMeshParser *_parser;
    ObjModel::Ptr _model;
    int nfid, fid, vroot, va, vb;
    bool _failed;
};  // end struct


// public
int ObjModelTriangleMeshParser::parse( int fid, const cv::Vec3d planev)
{
    assert(_parsedFaces);

    if ( fid < 0)
        fid = *_model->getFaceIds().begin();

    _parsedFaces->clear();
    std::stack<Triangle> *stack = new std::stack<Triangle>;

    const int* vindices = _model->getFaceVertices(fid);
    assert( vindices != NULL);
    int vroot = vindices[0];
    int va = vindices[1];

    // Decide parse ordering of face vertices?
    const double pvnorm = cv::norm(planev);
    if ( pvnorm > 0.0)
    {
        // Need to choose base vertices vroot and va to give normals that point
        const cv::Vec3d tstNorm = ObjModelNormals::calcNormal( _model, vroot, va, vindices[2]);
        cv::Vec3d u;
        cv::normalize( planev, u);
        // If the face normal calculated is not in the direction of planev, swap starting vertices.
        if ( u.dot(tstNorm) < 0)
            std::swap( vroot, va);
    }   // end if

    Triangle t( this, _model, fid, vroot, va);
    t.canLeft();
    t.canRight();
    stack->push( t);

    while ( !stack->empty())
    {
        t = stack->top();
        stack->pop();

        while ( !t.failed())
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
    return (int)_parsedFaces->size();
}   // end parse


// private
void ObjModelTriangleMeshParser::processTriangleParsers( int fid, int vr, int va, int vb)
{
    std::for_each( std::begin(_tparsers), std::end(_tparsers), [=]( ObjModelTriangleParser* t){ t->parseTriangle( fid, vr, va, vb);});
}   // end processTriangleParsers


// private
void ObjModelTriangleMeshParser::informFinishedParsing()
{
    std::for_each( std::begin(_tparsers), std::end(_tparsers), []( ObjModelTriangleParser* t){ t->finishedParsing();});
    if ( _bparser)
        _bparser->finishedParsing();
}   // end informFinishedParsing


// private
bool ObjModelTriangleMeshParser::parseEdge( int fid, int r, int a)
{
    bool v = true;
    if ( _bparser)
        v = _bparser->parseEdge( fid, r, a);
    return v;
}   // end parseEdge
