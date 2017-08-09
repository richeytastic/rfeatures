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


int setNewFaces( ObjModel::Ptr m, int fid, int u0, int u1, const cv::Vec3f& nv, int& neid)
{
    const int* v = m->getFaceVertices(fid);

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
        const int* uvis = m->getFaceUVs(fid);
        const int j = (i+1)%3;
        const int k = (i+2)%3;
        const cv::Vec2f& txtop = m->uv(matID, uvis[0]);
        const cv::Vec2f& tx0   = m->uv(matID, uvis[j]);
        const cv::Vec2f& tx1   = m->uv(matID, uvis[k]);
        const cv::Vec2f ntx = (tx0 + tx1) * 0.5f;

        const int vidxsA[3] = {vtop, v0, iv};
        const cv::Vec2f uvsA[3] = {txtop, tx0, ntx};
        m->setOrderedFaceUVs( matID, f0, vidxsA, uvsA);

        const int vidxsB[3] = {iv, v1, vtop};
        const cv::Vec2f uvsB[3] = {ntx, tx1, txtop};
        m->setOrderedFaceUVs( matID, f1, vidxsB, uvsB);
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
            m->removeFace(fid);
    }   // end while

    return nadded;
}   // end addVertices

