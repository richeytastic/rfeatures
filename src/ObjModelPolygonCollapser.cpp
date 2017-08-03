#include <ObjModelPolygonCollapser.h>
using RFeatures::ObjModel;
using RFeatures::ObjPoly;
using RFeatures::ObjModelPolygonCollapser;
#include <boost/foreach.hpp>
#include <cassert>
#include <cstdlib>


ObjModelPolygonCollapser::ObjModelPolygonCollapser( ObjModel::Ptr m) : _model(m) {}


// On all attached polygons, replace vertex rvid with vertex nvid
// (adjusting material texture offsets where needed).
void replaceVertex( ObjModel::Ptr m, int rvid, int nvid, int matId, const cv::Vec2f& tx)
{
    int newfid;
    int nvs[3];
    cv::Vec3i vis;
    cv::Vec6f txs;
    const IntSet cfs = m->getFaceIds( rvid);    // Copied out since removing polygons
    BOOST_FOREACH ( int fid, cfs)
    {
        const ObjPoly& face = m->getFace(fid);

        // Get the vertex IDs for the new face to add (identifying and replacing the removed vertex)
        memcpy( nvs, face.vindices, 3*sizeof(int));
        if ( nvs[0] == rvid)
            nvs[0] = nvid;
        else if ( nvs[1] == rvid)
            nvs[1] = nvid;
        else if ( nvs[2] == rvid)
            nvs[2] = nvid;

        if ( m->getFaceId( nvs[0], nvs[1], nvs[2]) >= 0)
        {
            const int ffid = m->getFaceId( nvs[0], nvs[1], nvs[2]);
            std::cerr << "Found face ID " << ffid << " with vertices " << nvs[0] << ", " << nvs[1] << ", " << nvs[2] << std::endl;
            std::cerr << "Looking to replace face " << fid << " with vertices "
                << face.vindices[0] << ", " << face.vindices[1] << ", " << face.vindices[2] << std::endl;
            std::cerr << "Replacing vertex " << rvid << " with new vertex " << nvid << std::endl;
            continue;
        }   // end if

        newfid = m->setFace( nvs);

        // If this polygon has the same material ID, set the new texture offsets.
        if ( matId >= 0 && m->getFaceMaterialId(fid) == matId)
        {
            vis = m->getMaterial(matId).faceVertexOrder.at(fid);  // Copied out
            txs = m->getMaterial(matId).txOffsets.at(fid);        // Copied out
            if ( vis[0] == rvid)
            {
                vis[0] = nvid;
                txs[0] = tx[0];
                txs[1] = tx[1];
            }   // end if
            else if ( vis[1] == rvid)
            {
                vis[1] = nvid;
                txs[2] = tx[0];
                txs[3] = tx[1];
            }   // end else if
            else if ( vis[2] == rvid)
            {
                vis[2] = nvid;
                txs[4] = tx[0];
                txs[5] = tx[1];
            }   // end else if

            assert( m->getFaceMaterialId( newfid) < 0);
            m->setOrderedFaceTextureOffsets( matId, newfid, &vis[0], (const cv::Vec2f*)&txs);
        }   // end if

        // Finally, remove the old polygon.
        m->unsetFace(fid);
    }   // end foreach
}   // end replaceVertex


void collapseEdge( ObjModel::Ptr m, int rv0, int rv1, int matId, const cv::Vec3i& tvis, const cv::Vec6f& tuvs)
{
    const cv::Vec3f nvert = ( m->vtx(rv0) + m->vtx(rv1)) * 1.0f/2;  // New interpolated vertex on edge
    const int nvidx = m->addVertex( nvert);
    cv::Vec2f tx2(0,0);
    if ( matId >= 0)
    {
        for ( int i = 0; i < 3; ++i)
            if ( tvis[i] == rv0 || tvis[i] == rv1)
                tx2 += cv::Vec2f( tuvs[2*i], tuvs[2*i+1]);
        tx2 *= 1.0f/2;
    }   // end if
    replaceVertex( m, rv0, nvidx, matId, tx2);
    replaceVertex( m, rv1, nvidx, matId, tx2);
    m->removeVertex( rv0);
    m->removeVertex( rv1);
}   // end collapseEdge


int ObjModelPolygonCollapser::collapse( int fid)
{
    ObjModel::Ptr m = _model;
    assert( m->getFaceIds().count(fid));
    if ( m->getFaceIds().count(fid) == 0)
        return -1;

    const ObjPoly& face = m->poly(fid);

    // If this polygon is not a part of a local triangulated mesh, cannot remove the it!
    if ( m->getNumSharedFaces( face.vindices[0], face.vindices[1]) > 2
      || m->getNumSharedFaces( face.vindices[1], face.vindices[2]) > 2
      || m->getNumSharedFaces( face.vindices[2], face.vindices[0]) > 2)
        return -2;

    int rvids[3];   // Copy out since removing the polygon
    memcpy( rvids, face.vindices, 3*sizeof(int));

    // Get the texture coordinates for the polygon being removed.
    // This will be used if adjacent polys with vertices being moved also have the same material (generally the case).
    const int matId = m->getFaceMaterialId( fid);
    cv::Vec3i tvis;
    cv::Vec6f tuvs;
    if ( matId >= 0)
    {
        tvis = m->getMaterial(matId).faceVertexOrder.at(fid);
        tuvs = m->getMaterial(matId).txOffsets.at(fid);
    }   // end if

    m->unsetFace(fid);  // Remove the polygon

    // If the polygon shares no other polygons with any of its edges, or shares polygons with only one
    // of its edges, collapsing the polygon is the same as removing it since no other polygons will be affected.
    const IntSet& sfids0 = m->getSharedFaces( rvids[0], rvids[1]);
    const IntSet& sfids1 = m->getSharedFaces( rvids[1], rvids[2]);
    const IntSet& sfids2 = m->getSharedFaces( rvids[2], rvids[0]);
    const size_t polyEdgeCount = sfids0.size() + sfids1.size() + sfids2.size();
    assert( polyEdgeCount < 4);

    // If the polygon shares vertices with all three of its edges, the polygon will be collapsed to a point.
    // This is the usual behaviour for the vast majority of polygons in a triangulated mesh.
    if ( polyEdgeCount == 3)
    {
        // Remove polygons that share edges of the removed polygon. The gaps left behind will be
        // filled when the 3 vertices of the removed polygon are reduced to a single newly inserted vertex.
        if ( !sfids0.empty()) m->unsetFace( *sfids0.begin());
        if ( !sfids1.empty()) m->unsetFace( *sfids1.begin());
        if ( !sfids2.empty()) m->unsetFace( *sfids2.begin());

        // The removed polygon's vertices are reduced to a new single vertex found as the mean of these vertices.
        const cv::Vec3f nvert = ( m->vtx(rvids[0]) + m->vtx(rvids[1]) + m->vtx(rvids[2])) * 1.0f/3;
        const int nvidx = m->addVertex( nvert);

        cv::Vec2f tx3;
        if ( matId >= 0)
            tx3 = cv::Vec2f((tuvs[0] + tuvs[2] + tuvs[4])/3, (tuvs[1] + tuvs[3] + tuvs[5])/3);

        // For the remaining polygons attached to rvids, the corresponding vertex needs to be replaced with nvidx
        replaceVertex( m, rvids[0], nvidx, matId, tx3);
        replaceVertex( m, rvids[1], nvidx, matId, tx3);
        replaceVertex( m, rvids[2], nvidx, matId, tx3);
        m->removeVertex( rvids[0]);
        m->removeVertex( rvids[1]);
        m->removeVertex( rvids[2]);
    }   // end if
    else if ( polyEdgeCount == 2)
    {
        // New vertex is will be positioned halfway along the edge that has no other polygons shared with it.
        if ( !sfids0.empty() && !sfids1.empty())
            collapseEdge( m, rvids[0], rvids[2], matId, tvis, tuvs);
        if ( !sfids1.empty() && !sfids2.empty())
            collapseEdge( m, rvids[0], rvids[1], matId, tvis, tuvs);
        if ( !sfids0.empty() && !sfids2.empty())
            collapseEdge( m, rvids[1], rvids[2], matId, tvis, tuvs);
    }   // end else if
    else if ( polyEdgeCount == 1)
    {
        // Need to remove the vertex opposite the edge that is being shared.
        if ( !sfids0.empty())
            m->removeVertex( rvids[2]);
        else if ( !sfids1.empty())
            m->removeVertex( rvids[0]);
        else if ( !sfids2.empty())
            m->removeVertex( rvids[1]);
    }   // end else if
    else
    {   // polyEdgeCount == 0
        // Polygon fid is a single isolated polygon so its vertices are no longer required.
        m->removeVertex( rvids[0]);
        m->removeVertex( rvids[1]);
        m->removeVertex( rvids[2]);
    }   // end else

    return polyEdgeCount;
}   // end collapse
