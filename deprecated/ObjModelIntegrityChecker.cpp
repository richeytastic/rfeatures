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

#include <ObjModelIntegrityChecker.h>
#include <ObjModelTopologyFinder.h>
#include <iostream>
using RFeatures::ObjModelIntegrityChecker;
using RFeatures::ObjModelTopologyFinder;
using RFeatures::ObjModel;


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
    _integrity = false;
}   // end reset



// public
std::ostream& RFeatures::operator<<( std::ostream& os, const ObjModelIntegrityChecker &ic)
{
    os << " [RFeatures::ObjModelIntegrityChecker]" << " INTEGRITY OKAY? " << std::boolalpha << ic.integrity() << std::endl;
    if ( ic.integrity())
    {
        const int nLine = ic.getNumLine();     // Connected to one other vertex
        const int n1Da = ic.getNumUnconnected();   // Lonely
        const int n1Db = ic.getNumFlatJunction();
        const int nNonFlatJunctionA = ic.getNumNonFlatJunctionAType();
        const int nNonFlatJunctionB = ic.getNumNonFlatJunctionBType();
        const int n3D = ic.getNumNonFlat();
        const int nbound = ic.getNumEdge();
        const int nfbound = ic.getNumFlatEdge();

        os << "  " << n1Da << " unconnected vertices" << std::endl;
        os << "  " << nLine << " connected vertices without polygons defined" << std::endl;
        os << "  " << n1Db << " flat junction vertices" << std::endl;
        os << "  " << nNonFlatJunctionA << " non-flat junction A vertices" << std::endl;
        os << "  " << nNonFlatJunctionB << " non-flat junction B vertices" << std::endl;
        os << "  " << n3D << " non-flat vertices" << std::endl;
        os << "  " << nbound << " boundary vertices, of which " << nfbound << " are flat" << std::endl;
        os << " Triangulated manifold? " << std::boolalpha << ic.is2DManifold() << std::endl;
    }   // end if
    return os;
}   // end operator<<


// public
bool ObjModelIntegrityChecker::checkIntegrity( const ObjModel* model)
{
    reset();

    ObjModelTopologyFinder omtf(model);
    const IntSet& vidxs = model->vertexIds();
    for ( int vidx : vidxs)
    {
        const bool isBoundary = omtf.isBoundary(vidx);
        if ( isBoundary)
            _edges.insert(vidx);

        const ObjModelTopologyFinder::BasicTopology btopology = omtf.getBasicTopology( vidx);
        if ( btopology & ObjModelTopologyFinder::VTX_UNCONNECTED)
        {
            _unconnected.insert(vidx);
            if ( !model->getFaceIds(vidx).empty())
            {
#ifndef NDEBUG
                std::cerr << "[ERROR] RFeatures::ObjModelIntegrityChecker::checkIntegrity: "
                          << "Topology says vertex not connected to any faces, but return value from model disagrees!" << std::endl;
#endif
                return false;
            }   // end if
        }   // end if
        else if ( btopology & ObjModelTopologyFinder::VTX_LINE)
            _line.insert(vidx);
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

        const IntSet& fids = model->getFaceIds(vidx);

        // For all of the vertices making up each face with vidx as one of its vertices,
        // check that they are connected directly to vidx
        for ( int fid : fids)
        {
            const int* vids = model->getFaceVertices(fid);
            if ( vids == nullptr)
            {
                std::cerr << "[ERROR] RFeatures::ObjModelIntegrityChecker::checkIntegrity: "
                    << "triangle " << fid << " has no stored vertices!" << std::endl;
                return false;
            }   // end if

            const int fv0 = vids[0];
            const int fv1 = vids[1];
            const int fv2 = vids[2];

            // Check that one of fv0, fv1, fv2 is vidx
            if ( fv0 != vidx && fv1 != vidx && fv2 != vidx)
            {
                std::cerr << "[ERROR] RFeatures::ObjModelIntegrityChecker::checkIntegrity: "
                    << "triangle " << fid << " does not have vertex " << vidx << " as one of its members!" << std::endl;
                return false;
            }   // end if

            // Check for duplicates
            if ( fv0 == fv1 || fv0 == fv2 || fv1 == fv2)
            {
                std::cerr << "[ERROR] RFeatures::ObjModelIntegrityChecker::checkIntegrity: "
                    << "triangle " << fid << " has duplicate vertices: " << fv0 << ", " << fv1 << ", " << fv2 << std::endl;
                return false;
            }   // end if
        }   // end foreach
    }   // end foreach

    const int nLine = getNumLine();     // Connected to one other vertex
    const int n1Da = getNumUnconnected();   // Lonely
    const int n1Db = getNumFlatJunction();
    const int nNonFlatJunctionA = getNumNonFlatJunctionAType();
    const int nNonFlatJunctionB = getNumNonFlatJunctionBType();
    const int n3D = getNumNonFlat();
    const int nbound = getNumEdge();
    const int nfbound = getNumFlatEdge();
    _is2DManifold = (nLine == 0) && (n1Da == 0) && (n1Db == 0)
        && (nNonFlatJunctionA == 0) && (nNonFlatJunctionB == 0) && (n3D == 0) && (nbound == nfbound);
    _integrity = true;
    return true;
}   // end checkIntegrity

