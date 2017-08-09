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

#include "ObjModelIntegrityChecker.h"
#include "ObjModelTopologyFinder.h"
using RFeatures::ObjModelIntegrityChecker;
using RFeatures::ObjModelIntegrityError;
using RFeatures::ObjModel;
#include <iostream>
#include <boost/foreach.hpp>


ObjModelIntegrityChecker::ObjModelIntegrityChecker( const ObjModel::Ptr& m) : _model(m)
{
    reset();
}   // end ctor


// private
void ObjModelIntegrityChecker::reset()
{
    _flat.clear();
    _nonFlat.clear();
    _unconnected.clear();
    _line.clear();
    _flatJunction.clear();
    _nonFlatJunctionA.clear();
    _nonFlatJunctionB.clear();
    _edges.clear();
    _flatEdges.clear();
    _is2DManifold = false;
}   // end reset


// private
bool ObjModelIntegrityChecker::checkIs2DManifold() const
{
    const int nLine = getNumLine();     // Connected to one other vertex
    const int n1Da = getNumUnconnected();   // Lonely
    const int n1Db = getNumFlatJunction();
    const int nNonFlatJunctionA = getNumNonFlatJunctionAType();
    const int nNonFlatJunctionB = getNumNonFlatJunctionBType();
    const int n3D = getNumNonFlat();
    const int nbound = getNumEdge();
    const int nfbound = getNumFlatEdge();
    return nLine == 0 && n1Da == 0 && n1Db == 0 && nNonFlatJunctionA == 0 && nNonFlatJunctionB == 0 && n3D == 0 && nbound == nfbound;
}   // end checkIs2DManifold



// public
std::ostream& operator<<( std::ostream& os, const ObjModelIntegrityChecker &ic)
{
    const int nLine = ic.getNumLine();     // Connected to one other vertex
    const int n1Da = ic.getNumUnconnected();   // Lonely
    const int n1Db = ic.getNumFlatJunction();
    const int nNonFlatJunctionA = ic.getNumNonFlatJunctionAType();
    const int nNonFlatJunctionB = ic.getNumNonFlatJunctionBType();
    const int n3D = ic.getNumNonFlat();
    const int nbound = ic.getNumEdge();
    const int nfbound = ic.getNumFlatEdge();

    os << " [RFeatures::ObjModelIntegrityChecker]" << std::endl;
    os << "  " << n1Da << " unconnected vertices" << std::endl;
    os << "  " << nLine << " connected vertices without polygons defined" << std::endl;
    os << "  " << n1Db << " flat junction vertices" << std::endl;
    os << "  " << nNonFlatJunctionA << " non-flat junction A vertices" << std::endl;
    os << "  " << nNonFlatJunctionB << " non-flat junction B vertices" << std::endl;
    os << "  " << n3D << " non-flat vertices" << std::endl;
    os << "  " << nbound << " edge vertices, of which " << nfbound << " are flat" << std::endl;
    os << " Triangulated manifold? " << std::boolalpha << ic.is2DManifold() << std::endl;
    return os;
}   // end operator<<



// public
enum ObjModelIntegrityError ObjModelIntegrityChecker::checkIntegrity()
{
    reset();

    const IntSet& faceIds = _model->getFaceIds();
    if ( _model->getNumFaces() != faceIds.size())
        return POLY_COUNT_MISMATCH;

    const IntSet& edgeIds = _model->getEdgeIds();
    if ( _model->getNumEdges() != edgeIds.size())
        return EDGE_COUNT_MISMATCH;

    using RFeatures::ObjModelTopologyFinder;
    ObjModelTopologyFinder omtf(_model);
    const IntSet& vidxs = _model->getVertexIds();
    BOOST_FOREACH ( int vidx, vidxs)
    {
        const bool isBoundary = omtf.isBoundary(vidx);
        if ( isBoundary)
            _edges.insert(vidx);

        const ObjModelTopologyFinder::BasicTopology btopology = omtf.getBasicTopology( vidx);
        if ( btopology & ObjModelTopologyFinder::VTX_UNCONNECTED)
        {
            _unconnected.insert(vidx);
            if ( !_model->getFaceIds(vidx).empty())
                return VERTEX_POLY_CONNECTION_ERROR;
        }   // end if
        else if ( btopology & ObjModelTopologyFinder::VTX_LINE)
        {
            _line.insert(vidx);
        }   // end else if
        else
        {
            const ObjModelTopologyFinder::ComplexTopology ctopology = omtf.getComplexTopology( vidx);
            if ( ctopology & ObjModelTopologyFinder::VTX_FLAT)
                _flat.insert(vidx);
            else
                _nonFlat.insert(vidx);

            if (!( ctopology & ObjModelTopologyFinder::VTX_COMPLETE))
            {
                if ( ctopology & ObjModelTopologyFinder::VTX_FLAT)
                    _flatJunction.insert(vidx);
                else if ( ctopology & ObjModelTopologyFinder::VTX_EDGE)
                    _nonFlatJunctionA.insert(vidx);
                else if ( ctopology & ObjModelTopologyFinder::VTX_JUNCTION_B)
                    _nonFlatJunctionB.insert(vidx);   // Two complete surfaces being joined
            }   // end if

            if ( ctopology & ObjModelTopologyFinder::VTX_FLAT && isBoundary)
                _flatEdges.insert(vidx);
        }   // end else

        const IntSet& fids = _model->getFaceIds(vidx);
        const IntSet& cuvtxs = _model->getConnectedVertices(vidx);   // Connected vertices to vidx

        BOOST_FOREACH ( const int& fid, fids)
        {
            // Check the vertex connections
            const int* vids = _model->getFaceVertices(fid);
            int fcount = 0;
            for ( int i = 0; i < 3; ++i)
            {
                if ( vids[i] == vidx)
                    fcount = 3;
                else if ( !cuvtxs.count(vids[i]))
                    fcount--;
            }   // end for

            if ( fcount != 3)
                return VERTEX_POLY_CONNECTION_ERROR;
        }   // end foreach
    }   // end foreach

    _is2DManifold = checkIs2DManifold();
    return NO_INTEGRITY_ERROR;
}   // end checkIntegrity

