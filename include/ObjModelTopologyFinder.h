/**
 * Implements an algorithm to analyse the connectivity of unique vertices and return
 * the local topological characteristics.
 */

#ifndef RFEATURES_OBJ_MODEL_TOPOLOGY_FINDER_H
#define RFEATURES_OBJ_MODEL_TOPOLOGY_FINDER_H

#include "ObjModel.h"

namespace RFeatures
{

class rFeatures_EXPORT ObjModelTopologyFinder
{
public:
    explicit ObjModelTopologyFinder( const ObjModel::Ptr&);
    virtual ~ObjModelTopologyFinder();

    // Given two edge vertices, find out if a polygon exists with its third point being
    // vertex reference checkVidx. Returns the ID of the polygon or -1 if one doesn't exist.
    int doesPolyExist( int edgeVidx0, int edgeVidx1, int checkVidx) const;

    enum BasicTopology
    {
        VTX_UNCONNECTED = 1, // x shares no edges with other vertices (is not connected to any other vertices).
        VTX_LINE = 2,        // x shares other vertices as part of a web of edges without polygons defined.
        VTX_TIP = 4,         // x is shared by a single polygon.
        VTX_COMPLEX = 8      // x has more complex topology - call getComplexTopology for info.
    };  // end enum

    BasicTopology getBasicTopology( int vidx) const;

    // In the following notes, E(x) denotes the set of vertices sharing edges with vertex x.
    // JUNCTION_A is a shared to a surface with x at the junction. Given when ComplexTopology == VTX_EDGE (i.e., not complete and not flat)
    enum ComplexTopology
    {
        VTX_JUNCTION_B = 1,  // x is a junction of two surfaces
        VTX_EDGE = 2,        // \exists y \in E(x) | edge xy is shared by exactly one polygon.
        VTX_COMPLETE = 4,    // \forall y,z \in E(x) | y!=x && z!=x there exists a path from y to z not via x.
        VTX_FLAT = 8         // In edge set E(x), all edges xy that share a single poly are members of H(x).
                             // If H(x) is empty and no edge xy in E(x) shares more than 2 polys, then x is FLAT.
                             // But if H(x) is not empty and x is FLAT, then starting at an edge xy \in H(x), it should
                             // be possible to account for all edges in E(x) as the single polygon shared by xy \in H(x)
                             // is iteratively removed (i.e., removal of the single polygon attached to edge xy causes
                             // edge xz to be added to H(x) until x no longer shares an edge with any other vertex).
    };  // end enum

    // Make sure that getBasicTopology() has returned VTX_COMPLEX, for the return value from this function to be valid.
    ComplexTopology getComplexTopology( int vidx) const;

    bool isBoundary( int vidx) const;

private:
    struct Impl;
    Impl* _impl;
};  // end class

}   // end namespace

#endif
