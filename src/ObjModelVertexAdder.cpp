#include "ObjModelVertexAdder.h"
using RFeatures::ObjModel;
using RFeatures::ObjPoly;
using RFeatures::Edge;
#include <boost/foreach.hpp>
#include <queue>

bool checkNeedNewFaces( const ObjModel::Ptr m, int eid, double minLen, cv::Vec3f& nv)
{
    const Edge& edge = m->getEdge( eid);
    const cv::Vec3f& v0 = m->getVertex( edge.v0);
    const cv::Vec3f& v1 = m->getVertex( edge.v1);
    const cv::Vec3f delta = v1-v0;
    bool needNew = false;
    if ( cv::norm(delta) > minLen)
    {
        nv = v0 + delta* 0.5f;
        needNew = true;
    }   // end if
    return needNew;
}   // end checkNeedNewFaces


// Get the face vertex indices for a face. If a material is
// associated with the face, get the vertices in the stored texture offset order.
// Returns true if material associated.
bool getFaceVertexIndices( const ObjModel::Ptr m, int fid, int& c0, int& c1, int& c2)
{
    const int matID = m->getFaceMaterialId( fid);
    if ( matID < 0)
    {
        const ObjPoly& face = m->getFace(fid);
        c0 = face.vindices[0];
        c1 = face.vindices[1];
        c2 = face.vindices[2];
    }   // end if
    else
    {   // Get the vertices in order that the texture vertices were set
        const ObjModel::Material& mat = m->getMaterial( matID);
        const cv::Vec3i& vindices = mat.faceVertexOrder.at(fid);
        c0 = vindices[0];
        c1 = vindices[1];
        c2 = vindices[2];
    }   // end else
    return matID >= 0;
}   // end getFaceVertexIndices


int setNewFaces( ObjModel::Ptr m, int fid, int u0, int u1, const cv::Vec3f& nv, int& neid)
{
    int v[3];
    getFaceVertexIndices( m, fid, v[0], v[1], v[2]);

    // Find the triangle vertex that is not at u0 or u1
    int i = -1;
    int tu;
    do {
        tu = v[++i];
    } while ( tu == u0 || tu == u1);

    const int vtop = v[i];
    const int v0 = v[(i+1)%3];
    const int v1 = v[(i+2)%3];

    const int iv = m->addVertex( nv);   // Lies on existing edge of two triangles

    const int f0 = m->setFace( vtop, v0, iv);
    const int f1 = m->setFace( vtop, v1, iv);

    const int matID = m->getFaceMaterialId( fid);
    if ( matID >= 0)
    {
        const ObjModel::Material& mat = m->getMaterial( matID);
        const cv::Vec6f& tx = mat.txOffsets.at(fid);

        const int j = 2*((i+1)%3);
        const int k = 2*((i+2)%3);
        const cv::Vec2f txtop( tx[0], tx[1]);
        const cv::Vec2f tx0( tx[j], tx[j+1]);
        const cv::Vec2f tx1( tx[k], tx[k+1]);
        const cv::Vec2f ntx = (tx0 + tx1) * 0.5f;

        m->setFaceTextureOffsets( matID, f0, vtop, txtop,
                                               v0, tx0,
                                               iv, ntx);

        m->setFaceTextureOffsets( matID, f1, iv, ntx,
                                             v1, tx1,
                                             vtop, txtop);
    }   // end if

    neid = m->getEdgeId( iv, vtop);
    return iv;
}   // end setNewFaces


// public static
int RFeatures::ObjModelVertexAdder::addVertices( ObjModel::Ptr m, double minLen)
{
    int nadded = 0;

    const IntSet& eset = m->getEdgeIds();
    std::queue<int> eids;
    BOOST_FOREACH ( const int& e, eset)
        eids.push(e);

    while ( !eids.empty())
    {
        const int eid = eids.front();
        eids.pop();

        cv::Vec3f nv;   // Will be position of new vertex to add if necessary
        if ( !checkNeedNewFaces( m, eid, minLen, nv))
            continue;

        // Need to adjust the two triangles
        const Edge& edge = m->getEdge( eid);
        const int ev0 = edge.v0;
        const int ev1 = edge.v1;
        const IntSet sfids = m->getSharedFaces( ev0, ev1);  // Copied out because modifying
        assert( sfids.size() <= 2); // Only for triangulated surfaces!

        int uv, neid;
        BOOST_FOREACH ( const int& fid, sfids)
        {
            uv = setNewFaces( m, fid, ev0, ev1, nv, neid);
            eids.push( neid);
            nadded++;
        }   // end foreach
        eids.push( m->getEdgeId( ev0, uv)); // Common to added faces
        eids.push( m->getEdgeId( ev1, uv)); // Common to added faces

        BOOST_FOREACH ( const int& fid, sfids)
            m->unsetFace(fid);
    }   // end while

    return nadded;
}   // end addVertices

