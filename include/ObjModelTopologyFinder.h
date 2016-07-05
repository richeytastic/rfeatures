#ifndef RFEATURES_OBJ_MODEL_TOPOLOGY_FINDER_H__
#define RFEATURES_OBJ_MODEL_TOPOLOGY_FINDER_H__

/**
 * Implements an algorithm to analyse the connectivity of unique vertices and return
 * the local topological characteristics.
 */

#include "ObjModel.h"

namespace RFeatures
{

class rFeatures_EXPORT ObjModelTopologyFinder
{
public:
    explicit ObjModelTopologyFinder( const ObjModel::Ptr&);
    virtual ~ObjModelTopologyFinder();

    // Return the vertex ID in poly that isn't v0 or v1 (works with
    // polygons that hold references to unique or to non unique vertex IDs).
    static int getOtherVertex( const ObjPoly& poly, int v0, int v1);

    // Non static version of the above that accepts a poly face ID and
    // uses the unique vertex reference version of the poly - so uvidx0 and uvidx1
    // must be unique vertex IDs.
    int getOtherVertex( int polyFaceId, int uvidx0, int uvidx1) const;

    // Given two edge vertices, find out if a polygon exists with its third
    // point being unique vertex reference checkUvidx. Returns the ID of the polygon
    // or -1 if one doesn't exist.
    int doesUniquePolyExist( int edgeUvidx0, int edgeUvidx1, int checkUvidx) const;

    enum VertexTopology
    {
        VTX_JUNCTION_B = 0,  // x joins at least two surfaces at a single point.
        VTX_UNCONNECTED = 1, // x shares no edges with other vertices (is not connected to any other vertices).
        VTX_COMPLETE = 2,    // In edge set E_x, \forall zy \in E_x there is a path from z to y not via x.
        VTX_EDGE = 4,        // In edge set E_x, \exists zy | zy is shared by exactly one polygon.
        VTX_FLAT = 8,        // In edge set E_x, all edges zy that share a single poly are members of H_x.
                             // If H_x is empty and no edge zy in E_x shares more than 2 polys, then x is FLAT.
                             // Alternatively, if H_x is not empty, then starting at some edge zy \in H_x, it
                             // is possible to iteratively account for all edges in E_x by moving them into H_x
                             // as the single polygon shared by zy at each iteration is removed. That is, the
                             // removal of the single shared polygon always exposes a new member of H_x until
                             // all members of E_x are accounted for.
    };  // end enum
    VertexTopology discoverLocalTopology( int uvidx) const;

    bool isBoundary( int uvidx) const;

private:
    struct Impl;
    Impl* _impl;
};  // end class

}   // end namespace

#endif
