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
    const IntSet& vidxs = model->getVertexIds();
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
        const IntSet& cvtxs = model->getConnectedVertices(vidx);   // vertices connected to vidx

        // For all of the vertices making up each face with vidx as one of its vertices,
        // check that they are connected directly to vidx
        for ( int fid : fids)
        {
            const int* vids = model->getFaceVertices(fid);
            assert(vids);
            bool gotVidx = false;
            for ( int i = 0; i < 3; ++i)
            {
                if ( vids[i] == vidx)
                {
                    if ( !gotVidx)
                        gotVidx = true;
                    else
                    {
                        std::cerr << "[ERROR] RFeatures::ObjModelIntegrityChecker::checkIntegrity: "
                            << "Found a face with ID=" << fid << " having at least one pair of duplicate vertex IDs ("
                            << vidx << ")!" << std::endl;
                        return false;
                    }   // end else
                }   // end if
                else
                {
                    if ( cvtxs.count(vids[i]) == 0)
                    {
                        std::cerr << "[ERROR] RFeatures::ObjModelIntegrityChecker::checkIntegrity: "
                            << "Found vertex B with ID=" << vids[i] << " that doesn't appear to be connected "
                            << "in the model to vertex A with ID=" << vidx << ", even though vertex B is stored "
                            << "as one of the vertices of face with ID=" << fid << " which is stored as being a "
                            << "face that uses vertex A!" << std::endl;
                        return false;
                    }   // end if
                }   // end else
            }   // end for

            if ( !gotVidx)
            {
                std::cerr << "[ERROR] RFeatures::ObjModelIntegrityChecker::checkIntegrity: "
                    << "Failed to find vertex A with ID=" << vidx << " in the vertices stored as belonging "
                    << "to a face with ID=" << fid << " which is stored as being connected to vertex A!" << std::endl;
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

