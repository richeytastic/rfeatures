#include <ObjModel.h>
#include <cmath>
#include <cstring>
#include <iostream>
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <cassert>
#include <algorithm>
#include <boost/foreach.hpp>
#include <boost/functional/hash.hpp>
#include <FeatureUtils.h>

using RFeatures::ObjPoly;
using RFeatures::Edge;
using RFeatures::ObjModel;


void reorderMinToMax( int &v0, int &v1, int &v2)
{
    if ( v0 > v1)
        std::swap( v0,v1);
    if ( v1 > v2)
        std::swap( v1,v2);
    if ( v0 > v1)
        std::swap( v0,v1);
}   // end reorderMinToMax


ObjPoly::ObjPoly() {}

ObjPoly::ObjPoly( int v0, int v1, int v2)
{
    reorderMinToMax( v0, v1, v2);
    vindices[0] = v0;
    vindices[1] = v1;
    vindices[2] = v2;
}   // end ctor


// Two ObjPolys are the same if they share the same vertices
bool ObjPoly::operator==( const ObjPoly& p) const
{
    return (vindices[0] == p.vindices[0]) && (vindices[1] == p.vindices[1]) && (vindices[2] == p.vindices[2]);
}   // end operator==


// public
bool ObjPoly::getOpposite( int vid, int& other0, int& other1) const
{
    bool found = true;

    if ( vid == vindices[0])
    {
        other0 = vindices[1];
        other1 = vindices[2];
    }   // end if
    else if ( vid == vindices[1])
    {
        other0 = vindices[0];
        other1 = vindices[2];
    }   // end else if
    else if ( vid == vindices[2])
    {
        other0 = vindices[0];
        other1 = vindices[1];
    }   // end else if
    else
        found = false;

    return found;
}   // end getOpposite


// public
int ObjPoly::getOpposite( int v0, int v1) const
{
    int vn = vindices[0];
    if ( vn == v0 || vn == v1)
    {
        vn = vindices[1];
        if ( vn == v0 || vn == v1)
            vn = vindices[2];
    }   // end if
    return vn;
}   // end getOpposite


Edge::Edge() {}

Edge::Edge( int u0, int u1) : v0(u0), v1(u1)
{
    assert( v0 != v1);
    if ( v1 < v0)   // Ensure v0 is always smaller
        std::swap(v0,v1);
}   // end ctor

bool Edge::operator==( const Edge& e) const
{
    return e.v0 == v0 && e.v1 == v1;
}   // end operator==


size_t RFeatures::HashObjPoly::operator()( const ObjPoly& u) const
{
    size_t seed = 0;
    boost::hash_combine( seed, u.vindices[0]);
    boost::hash_combine( seed, u.vindices[1]);
    boost::hash_combine( seed, u.vindices[2]);
    return seed;
}   // end operator()


size_t RFeatures::HashEdge::operator()( const Edge& u) const
{
    size_t seed = 0;
    boost::hash_combine( seed, u.v0);
    boost::hash_combine( seed, u.v1);
    return seed;
}   // end operator()


// public static
ObjModel::Ptr ObjModel::copy( const ObjModel::Ptr omc, bool shareMaterials)
{
    ObjModel::Ptr nm = create( omc->_fltPrc);

    // Copy over the vertices from the old model
    boost::unordered_map<int,int>* iimap = new boost::unordered_map<int,int>;
    const IntSet& vids = omc->getVertexIds();
    BOOST_FOREACH ( int vid, vids)
    {
        // Map the old vertex ID from the old model to the new vertex ID on the new model
        // since we want to relate the old indices in the polygons of the old model to the
        // new vertex indices on the new model.
        (*iimap)[vid] = nm->addVertex( omc->getVertex(vid));
    }   // end foreach

    const int nmats = (int)omc->getNumMaterials();
    for ( int i = 0; i < nmats; ++i)
    {
        const Material& m = omc->getMaterial(i);
        nm->addMaterial();

        if ( shareMaterials)
        {
            BOOST_FOREACH ( const cv::Mat& img, m.ambient)
                nm->addMaterialAmbient( i, img);
            BOOST_FOREACH ( const cv::Mat& img, m.diffuse)
                nm->addMaterialDiffuse( i, img);
            BOOST_FOREACH ( const cv::Mat& img, m.specular)
                nm->addMaterialSpecular( i, img);
        }   // end if
        else
        {
            BOOST_FOREACH ( const cv::Mat& img, m.ambient)
                nm->addMaterialAmbient( i, img.clone());
            BOOST_FOREACH ( const cv::Mat& img, m.diffuse)
                nm->addMaterialDiffuse( i, img.clone());
            BOOST_FOREACH ( const cv::Mat& img, m.specular)
                nm->addMaterialSpecular( i, img.clone());
        }   // end else
    }   // end for

    // Copy over the faces from the old model ensuring that the old vertex IDs from
    // the old model map to the new vertex IDs in the new model.
    const IntSet& fids = omc->getFaceIds();
    BOOST_FOREACH ( int fid, fids)
    {
        const ObjPoly& face = omc->getFace(fid);
        const int newFaceId = nm->setFace( iimap->at(face.vindices[0]), iimap->at(face.vindices[1]), iimap->at(face.vindices[2]));
        const int matId = omc->getFaceMaterialId( fid);
        if ( matId >= 0)
        {
            const Material& m = omc->getMaterial(matId);
            const cv::Vec3i& orderedVertices = m.faceVertexOrder.at(fid);
            const cv::Vec6f& tx = m.txOffsets.at(fid);
            nm->setFaceTextureOffsets( matId, newFaceId, iimap->at(orderedVertices[0]), cv::Vec2f( tx[0], tx[1]),
                                                         iimap->at(orderedVertices[1]), cv::Vec2f( tx[2], tx[3]),
                                                         iimap->at(orderedVertices[2]), cv::Vec2f( tx[4], tx[5]));
        }   // end if
    }   // end foreach

    delete iimap;
    return nm;
}   // end copy


// public
ObjModel::Ptr ObjModel::clone( bool shareMats)
{
    return ObjModel::copy( Ptr(this), shareMats);
}   // end clone


// public static
ObjModel::Ptr ObjModel::create( int fltPrc)
{
    return ObjModel::Ptr( new ObjModel(fltPrc));
}   // end create


// private
ObjModel::ObjModel( int fltPrc)
    :  _fltPrc( fltPrc), _vCounter(0), _faceCounter(0), _edgeCounter(0), _materialCounter(0)
{}   // end ctor


// public
int ObjModel::getFaceMaterialId( int fid) const
{
    assert( _faceIds.count(fid));
    if ( !_faceMaterial.count(fid)) // No material for this face
        return -1;
    return _faceMaterial.at(fid);
}   // end getFaceMaterialId


// public
const ObjModel::Material& ObjModel::getMaterial( int mid) const
{
    assert( _materials.count(mid));
    return _materials.at(mid);
}   // end getMaterial


// public
int ObjModel::addMaterial()
{
    const int mid = _materialCounter++;
    _materialIds.insert(mid);
    _materials[mid] = Material();
    return mid;
}   // end addMaterial


// public
bool ObjModel::removeMaterial( int materialID)
{
    if ( _materials.count(materialID) == 0)
        return false;
    _materialIds.erase(materialID);
    _materials.erase(materialID);
    return true;
}   // end removeMaterial


// public
bool ObjModel::removeAllMaterials()
{
    if ( _materialIds.empty())
        return false;
    _materialIds.clear();
    _materials.clear();
    return true;
}   // end removeAllMaterials


// public
bool ObjModel::addMaterialAmbient( int materialID, const cv::Mat& m)
{
    if ( _materials.count( materialID) == 0 || m.empty())
        return false;
    _materials[materialID].ambient.push_back( m);
    return true;
}   // end addMaterialAmbient


// public
bool ObjModel::addMaterialDiffuse( int materialID, const cv::Mat& m)
{
    if ( _materials.count( materialID) == 0 || m.empty())
        return false;
    _materials[materialID].diffuse.push_back( m);
    return true;
}   // end addMaterialDiffuse


// public
bool ObjModel::addMaterialSpecular( int materialID, const cv::Mat& m)
{
    if ( _materials.count( materialID) == 0 || m.empty())
        return false;
    _materials[materialID].specular.push_back( m);
    return true;
}   // end addMaterialSpecular


// private
void ObjModel::unsetFaceTextureOffsets( int matid, int fid)
{
    assert( getMaterialIds().count( matid));
    Material& m = _materials.at( matid);
    assert( m.faceVertexOrder.count(fid) == 1);
    m.faceVertexOrder.erase(fid);
    m.txOffsets.erase(fid);
    _faceMaterial.erase(fid);
}   // end unsetFaceTextureOffsets


// public
int ObjModel::addVertex( double x, double y, double z)
{
    if ( cvIsNaN( x) || cvIsNaN( y) || cvIsNaN( z))
        return -1;

    Key3L key = toKey( x, y, z, _fltPrc);
    if ( !_verticesToUniqIdxs.count( key))
        _verticesToUniqIdxs[key] = _vCounter++;

    const int vi = _verticesToUniqIdxs.at(key);
    if ( !_vtxIds.count(vi))
    {
        _vtxIds.insert(vi);
        _verts[vi] = cv::Vec3f( x, y, z);
        _vtxConnections[vi].clear();
        _vtxConnectionFaces[vi].clear();
        _vtxToFaces[vi].clear();
        _vtxToEdges[vi].clear();
    }   // end if
    return vi;
}   // end addVertex


// public
int ObjModel::addVertex( const cv::Vec3f& v)
{
    return addVertex( v[0], v[1], v[2]);
}   // end addVertex


// public
bool ObjModel::removeVertex( int vi)
{
    assert( getVertexIds().count(vi));

    // Check that this vertex is no longer connected to others
    // i.e., connecting faces have been removed.
    const IntSet& cvtxs = getConnectedVertices(vi);
    assert(cvtxs.empty());
    if ( !cvtxs.empty())
        return false;

    assert( getFaceIds(vi).empty());

    Key3L key = toKey( _verts[vi], _fltPrc);
    _verticesToUniqIdxs.erase(key);
    _vtxIds.erase(vi);
    _verts.erase(vi);
    _vtxConnections.erase(vi);
    _vtxConnectionFaces.erase(vi);
    _vtxToFaces.erase(vi);
    _vtxToEdges.erase(vi);

    return true;
}   // end removeVertex


// public
bool ObjModel::adjustVertex( int vidx, double x, double y, double z)
{
    if ( cvIsNaN( x) || cvIsNaN( y) || cvIsNaN( z))
        return false;

    if ( _vtxIds.count(vidx) == 0)
        return false;

    _verticesToUniqIdxs.erase( toKey( _verts[vidx], _fltPrc)); // Remove original vertex hash value
    _verts[vidx] = cv::Vec3f( x, y, z);
    _verticesToUniqIdxs[ toKey( _verts[vidx], _fltPrc)] = vidx;  // Hash back with new vertices
    return true;
}   // end adjustVertex


// public
bool ObjModel::adjustVertex( int vidx, const cv::Vec3f& v)
{
    return adjustVertex( vidx, v[0], v[1], v[2]);
}   // end adjustVertex


// public
int ObjModel::getVertexFaceCount( int vid) const
{
    assert( _vtxToFaces.count(vid) > 0);
    return (int)_vtxToFaces.at(vid).size();
}   // end getVertexFaceCount


// public
bool ObjModel::hasEdge( int v0, int v1) const
{
    const bool gotEdge = getConnectedVertices( v0).count(v1) > 0;
    assert( (gotEdge && getEdgeId(v0,v1) >= 0) || (!gotEdge && getEdgeId(v0,v1) < 0));
    return gotEdge;
}   // end hasEdge


// public
int ObjModel::getEdgeId( int v0, int v1) const
{
    const Edge lookupEdge( v0, v1);
    if ( _edgeMap.count(lookupEdge))
        return _edgeMap.at(lookupEdge);
    return -1;
}   // end getEdgeId


// private
int ObjModel::connectEdge( int v0, int v1)
{
    assert( v0 != v1);
    int eidx = getEdgeId( v0, v1);
    if ( eidx >= 0)
        return eidx;

    _vtxConnections[v0].insert(v1);
    _vtxConnections[v1].insert(v0);

    eidx = _edgeCounter++;
    _edgeMap[_edges[eidx] = Edge( v0, v1)] = eidx;
    _edgeIds.insert(eidx);
    _vtxToEdges[v0].insert(eidx);
    _vtxToEdges[v1].insert(eidx);
    return eidx;
}   // end connectEdge


// public
int ObjModel::setEdge( int v0, int v1)
{
    int eidx = getEdgeId( v0, v1);
    if ( eidx >= 0)
        return eidx;

    // If there are vertices T currently connected to v0 that are also connected to v1,
    // then connecting v0 and v1 directly will create new polygons between the
    // currently connected vertices T and v0 and v1.
    const IntSet& u0Conn = getConnectedVertices( v0);
    const IntSet& u1Conn = getConnectedVertices( v1);
    std::vector<int> ucs;

    // Iterate over smaller set
    const IntSet* uset = &u0Conn;
    const IntSet* sset = &u1Conn;
    if ( u1Conn.size() < u0Conn.size())
    {
        uset = &u1Conn;
        sset = &u0Conn;
    }   // end if
    BOOST_FOREACH ( int ui, *uset)
    {
        if ( sset->count(ui))
            ucs.push_back(ui);
    }   // end foreach

    const int edgeId = connectEdge( v0, v1);
    // Create the faces
    BOOST_FOREACH ( int v, ucs)
        setFace( v, v0, v1);
    return edgeId;
}   // end setEdge


// public
int ObjModel::setFace( const int* vtxs)
{
    return setFace( vtxs[0], vtxs[1], vtxs[2]);
}   // end setFace


// public
int ObjModel::getFaceId( int v0, int v1, int v2) const
{
    const ObjPoly lookupPoly( v0, v1, v2);
    if ( _faceMap.count(lookupPoly))
        return _faceMap.at(lookupPoly);
    return -1;
}   // end getFaceId


// public
int ObjModel::setFace( int v0, int v1, int v2)
{
    if ( getVertexIds().count(v0) == 0 || getVertexIds().count(v1) == 0 || getVertexIds().count(v2) == 0)
        return -1;

    int faceIdx = getFaceId( v0, v1, v2); // Don't add if already present
    if ( faceIdx >= 0)
        return faceIdx;

    faceIdx = _faceCounter++;

    const int e0 = connectEdge( v0, v1);
    _faceEdgeIdxs[faceIdx].insert( e0);
    _edgesToFaces[e0].insert(faceIdx);

    const int e1 = connectEdge( v1, v2);
    _faceEdgeIdxs[faceIdx].insert( e1);
    _edgesToFaces[e1].insert(faceIdx);

    const int e2 = connectEdge( v2, v0);
    _faceEdgeIdxs[faceIdx].insert( e2);
    _edgesToFaces[e2].insert(faceIdx);

    setVertexFaceConnections( faceIdx, v0, v1, v2);
    _faceIds.insert( faceIdx);

    _faceMap[_faces[faceIdx] = ObjPoly( v0, v1, v2)] = faceIdx;
    return faceIdx;
}   // end setFace


// public
bool ObjModel::setFaceTextureOffsets( int materialID, int fid, int v0, const cv::Vec2f& uv0,
                                                               int v1, const cv::Vec2f& uv1,
                                                               int v2, const cv::Vec2f& uv2)
{
    if ( _materials.count(materialID) == 0)
        return false;

    Material& mat = _materials[materialID];
#ifndef NDEBUG
    assert( getFaceIds().count(fid) > 0);
    assert( getVertexIds().count(v0) && getVertexIds().count(v1) && getVertexIds().count(v2));
    assert( !cvIsNaN( uv0[0]) && !cvIsNaN( uv0[1]));
    assert( !cvIsNaN( uv1[0]) && !cvIsNaN( uv1[1]));
    assert( !cvIsNaN( uv2[0]) && !cvIsNaN( uv2[1]));
    // Can't have added previously (no overwrites)
    assert( !mat.faceVertexOrder.count(fid));
    assert( !mat.txOffsets.count(fid));
    assert( !_faceMaterial.count(fid));
#endif
    mat.faceVertexOrder[fid] = cv::Vec3i( v0, v1, v2);
    mat.txOffsets[fid] = cv::Vec6f( uv0[0], uv0[1], uv1[0], uv1[1], uv2[0], uv2[1]);
    _faceMaterial[fid] = materialID;
    return true;
}   // end setFaceTextureOffsets


// private
void ObjModel::setVertexFaceConnections( int faceIdx, int v0, int v1, int v2)
{
    _vtxToFaces[v0].insert(faceIdx);
    _vtxToFaces[v1].insert(faceIdx);
    _vtxToFaces[v2].insert(faceIdx);

    _vtxConnectionFaces[v0][v1].insert(faceIdx);
    _vtxConnectionFaces[v0][v2].insert(faceIdx);
    _vtxConnectionFaces[v1][v0].insert(faceIdx);
    _vtxConnectionFaces[v1][v2].insert(faceIdx);
    _vtxConnectionFaces[v2][v0].insert(faceIdx);
    _vtxConnectionFaces[v2][v1].insert(faceIdx);
}   // end setUniqueVertexFaceConnections


// private
void ObjModel::unsetVertexFaceConnections( int faceIdx, int v0, int v1, int v2)
{
    _vtxToFaces[v0].erase(faceIdx);
    _vtxToFaces[v1].erase(faceIdx);
    _vtxToFaces[v2].erase(faceIdx);

    _vtxConnectionFaces[v0][v1].erase(faceIdx);
    _vtxConnectionFaces[v0][v2].erase(faceIdx);
    _vtxConnectionFaces[v1][v0].erase(faceIdx);
    _vtxConnectionFaces[v1][v2].erase(faceIdx);
    _vtxConnectionFaces[v2][v0].erase(faceIdx);
    _vtxConnectionFaces[v2][v1].erase(faceIdx);

    // Remove the entries if the connection between vertices shares no more faces,
    if ( _vtxConnectionFaces[v0][v1].empty())
    {
        _vtxConnectionFaces[v0].erase(v1);
        _vtxConnectionFaces[v1].erase(v0);
    }   // end if
    if ( _vtxConnectionFaces[v1][v2].empty())
    {
        _vtxConnectionFaces[v1].erase(v2);
        _vtxConnectionFaces[v2].erase(v1);
    }   // end if
    if ( _vtxConnectionFaces[v2][v0].empty())
    {
        _vtxConnectionFaces[v2].erase(v0);
        _vtxConnectionFaces[v0].erase(v2);
    }   // end if
}   // end unsetVertexFaceConnections


// private
void ObjModel::removeEdge( int eidx)
{
    const Edge& edge = getEdge(eidx);
    _vtxToEdges[edge.v0].erase(eidx);
    _vtxToEdges[edge.v1].erase(eidx);
    _vtxConnections[edge.v0].erase(edge.v1);
    _vtxConnections[edge.v1].erase(edge.v0);
    _edgeIds.erase(eidx);
    _edgeMap.erase(_edges.at(eidx));
    _edges.erase(eidx);
    _edgesToFaces.erase(eidx);
}   // end removeEdge


// private
void ObjModel::removeFaceEdges( int faceIdx)
{
    assert( _faceEdgeIdxs.count(faceIdx));
    const IntSet edgeIds = _faceEdgeIdxs.at(faceIdx);   // Copy out
    BOOST_FOREACH ( int eidx, edgeIds)
    {
        _faceEdgeIdxs[faceIdx].erase(eidx);
        _edgesToFaces[eidx].erase(faceIdx);
        // An edge is only removed if it has no more faces in common
        if ( _edgesToFaces.at(eidx).empty())
            removeEdge( eidx);
    }   // end foreach
    _faceEdgeIdxs.erase(faceIdx);
}   // end removeFaceEdges


// public
bool ObjModel::unsetEdge( int edgeIdx)
{
    if ( !_edgeIds.count(edgeIdx))
        return false;

    const Edge& e = getEdge( edgeIdx);
    // Removing an edge removes all attached faces
    const IntSet fids = getSharedFaces( e.v0, e.v1);    // Copy out
    BOOST_FOREACH ( int fid, fids)
        unsetFace( fid);    // May remove the edge if edge is shared by just one face

    if ( _edgeIds.count(edgeIdx))
        removeEdge( edgeIdx);

    return true;
}   // end unsetEdge


// public
bool ObjModel::unsetFace( int faceIdx)
{
    assert( _faces.count(faceIdx) > 0);
    if ( _faces.count(faceIdx) == 0)
        return false;

    const ObjPoly& face = getFace(faceIdx);
    const int v0 = face.vindices[0];
    const int v1 = face.vindices[1];
    const int v2 = face.vindices[2];

    unsetVertexFaceConnections( faceIdx, v0, v1, v2);
    removeFaceEdges( faceIdx);
    const int matid = getFaceMaterialId( faceIdx);

    _faceMap.erase(_faces.at(faceIdx));
    _faces.erase(faceIdx);
    _faceIds.erase(faceIdx);
 
    // Remove from the Material if present
    if ( matid >= 0)
        unsetFaceTextureOffsets( matid, faceIdx);
    return true;
}   // end unsetFace


// public
const ObjPoly& ObjModel::getFace( int faceId) const
{
    assert( _faceIds.count(faceId));
    return _faces.at(faceId);
}   // end getFace


// public
const ObjPoly& ObjModel::poly( int fid) const
{
    return _faces.at(fid);
}   // end poly


// public
int ObjModel::getFaceConnectivityMetric( int faceId) const
{
    if ( !_faceIds.count(faceId))
        return -1;

    const ObjPoly& face = getFace( faceId);
    int csum = -3;
    csum += (int)getFaceIds( face.vindices[0]).size();
    csum += (int)getFaceIds( face.vindices[1]).size();
    csum += (int)getFaceIds( face.vindices[2]).size();
    // Remove 1 for each edge shared with a second face
    if ( getNumSharedFaces( face.vindices[0], face.vindices[1]) > 1)
        csum--;
    if ( getNumSharedFaces( face.vindices[1], face.vindices[2]) > 1)
        csum--;
    if ( getNumSharedFaces( face.vindices[2], face.vindices[0]) > 1)
        csum--;
    return csum;
}   // end getFaceConnectivityMetric


// public
const Edge& ObjModel::getEdge( int edgeId) const
{
    assert( _edges.count(edgeId));
    return _edges.at(edgeId);
}   // end getEdge


// public
const IntSet& ObjModel::getEdgeIds( int vi) const
{
    return _vtxToEdges.at(vi);
}   // end getEdgeIds


// public
const IntSet& ObjModel::getConnectedVertices( int vi) const
{
    return _vtxConnections.at(vi);
}   // end getConnectedVertices


// public
size_t ObjModel::findAdjacentFaces( int fid, IntSet& fids) const
{
    assert( _faceIds.count(fid));
    fids.clear();
    const int* vidxs = getFace(fid).vindices;

    const IntSet& sfaces0 = getSharedFaces( vidxs[0], vidxs[1]);
    fids.insert( sfaces0.begin(), sfaces0.end());

    const IntSet& sfaces1 = getSharedFaces( vidxs[1], vidxs[2]);
    fids.insert( sfaces1.begin(), sfaces1.end());

    const IntSet& sfaces2 = getSharedFaces( vidxs[2], vidxs[0]);
    fids.insert( sfaces2.begin(), sfaces2.end());

    fids.erase(fid);    // Ensure the reference face id is excluded
    return fids.size();
}   // end findAdjacentFaces


// public
int ObjModel::getNumSharedFaces( int vi, int vj) const
{
    return (int)getSharedFaces( vi, vj).size();
}   // end getNumSharedFaces


// public
const IntSet& ObjModel::getSharedFaces( int vi, int vj) const
{
    if ( !_vtxConnectionFaces.count(vi))
        return _emptyIntSet;
    const boost::unordered_map<int, IntSet>& uicf = _vtxConnectionFaces.at(vi);
    if ( !uicf.count(vj))
        return _emptyIntSet;
    return uicf.at(vj);
}   // end getSharedFaces


// public
cv::Vec3f ObjModel::projectToPoly( int fid, const cv::Vec3f& v) const
{
    using namespace RFeatures;
    const ObjPoly& poly = getFace(fid);
    const cv::Vec3f& v0 = vtx( poly.vindices[0]);
    const cv::Vec3f& v1 = vtx( poly.vindices[1]);
    const cv::Vec3f& v2 = vtx( poly.vindices[2]);

    const cv::Vec3f v10 = v1 - v0;
    const cv::Vec3f v20 = v2 - v0;
    cv::Vec3f u1, u2;
    cv::normalize( v10, u1);
    cv::normalize( v20, u2);

    cv::Vec3f dv = v - v0;  // Difference vector of v with v0

    cv::Vec3f p1 = dv.dot(u1) * u1; // Projection of dv along unit vector of v1-v0
    // If the projection of v along u1 is longer than length of v10, set n1 to be v1 so as to constrain the length
    // of the projection so that we don't calculate a new projected position outside the bounds of the polygon.
    if ( l2sq( p1) > l2sq(v10))
        p1 = v10;

    // Same for projection of v along u.
    cv::Vec3f p2 = dv.dot(u2) * u2; // Projection of dv along unit vector of v2-v0
    if ( l2sq( p2) > l2sq(v20))
        p2 = v20;

    const cv::Vec3f n1 = v0 + p1;   // New psuedo vertex along edge v1-v0
    const cv::Vec3f n2 = v0 + p2;   // New psuedo vertex along edge v2-v0
    const cv::Vec3f n21 = n2 - n1;  // New psuedo edge
    cv::Vec3f nu;
    cv::normalize( n21, nu);        // Unit vector of new psuedo edge (may coincide with edge v2-v1 if projections constrained above)

    dv = v - n1;    // Difference vector of v with psuedo vertex n1
    // Constrain projection along unit vector of new psuedo edge to be within length of the psuedo edge.
    cv::Vec3f pn = dv.dot(nu) * nu;
    if ( l2sq( pn) > l2sq(n21))
        pn = n21;

    return n1 + pn; // Returned psuedo vertex is constrained projection along the psuedo edge.
}   // end projectToPoly


// public
bool ObjModel::isVertexInsideFace( int fid, const cv::Vec3f& v) const
{
    const ObjPoly& face = getFace( fid);
    const cv::Vec3f& va = getVertex( face.vindices[0]);
    const cv::Vec3f& vb = getVertex( face.vindices[1]);
    const cv::Vec3f& vc = getVertex( face.vindices[2]);

    const double a = cv::norm( vb - vc);
    const double b = cv::norm( va - vc);
    const double c = cv::norm( va - vb);

    const double ia = cv::norm( va - v);
    const double ib = cv::norm( vb - v);
    const double ic = cv::norm( vc - v);

    using namespace RFeatures;
    return calcTriangleArea( a,ib,ic) + calcTriangleArea( b,ia,ic) + calcTriangleArea( c,ia,ib) == calcTriangleArea( a,b,c);
}   // end isVertexInsideFace


// public
const IntSet& ObjModel::getFaceIds( int vi) const
{
    if ( !_vtxToFaces.count(vi))
        return _emptyIntSet;
    return _vtxToFaces.at(vi);
}   // end getFaceIds


// public
int ObjModel::lookupVertexIndex( const cv::Vec3f& v) const
{
    int vidx = -1;
    Key3L key = toKey( v, _fltPrc);
    if ( _verticesToUniqIdxs.count( key))
        vidx = _verticesToUniqIdxs.at(key);   // Get the unique vertex ID
    return vidx;
}   // end lookupVertexIndex


// public
void ObjModel::showDebug( bool withDetails) const
{
    // Print vertex info
    std::cerr << "===============[ RFeatures::ObjModel ]===============" << std::endl;
    const IntSet& vids = getVertexIds();
    const IntSet& fids = getFaceIds();

    if ( withDetails)
    {
        std::cerr << " Vertices:" << std::endl;
        BOOST_FOREACH ( int vid, vids)
        {
            const cv::Vec3f& v = getVertex( vid);
            std::cerr << "\tVTX_" << vid << "): x=" << v[0] << ", y=" << v[1] << ", z=" << v[2] << " [VTX connections:";
            // Show connected vertices
            const IntSet& cvs = getConnectedVertices( vid);
            BOOST_FOREACH ( int cv, cvs)
                std::cerr << " " << cv;
            std::cerr << "]" << std::endl;
        }   // end for
        std::cerr << std::endl;
    }   // end if

    if ( withDetails)
    {
        std::cerr << " Face vertex indices:" << std::endl;
        BOOST_FOREACH ( int fid, fids)
        {
            const ObjPoly& f = getFace(fid);
            std::cerr << "\tF_" << fid << "): " << f.vindices[0] << ", " << f.vindices[1] << ", " << f.vindices[2] << std::endl;
        }   // end for
        std::cerr << std::endl;
        std::cerr << " --------------------------------------------------- " << std::endl;
    }   // end if

    const IntSet& matIds = getMaterialIds();
    std::cerr << " Model has " << matIds.size() << " materials" << std::endl;
    BOOST_FOREACH ( int matId, matIds)
    {
        const Material& mat = getMaterial(matId);
        std::cerr << " Material " << matId << " has [" << mat.ambient.size() << "," << mat.diffuse.size() << "," << mat.specular.size()
                  << "] ambient,diffuse,specular maps" << std::endl;
        std::cerr << " Material texture maps " << mat.txOffsets.size() << " triangles" << std::endl;
        if ( withDetails)
        {
            typedef std::pair<int, cv::Vec6f> FaceTxOffsets;
            BOOST_FOREACH ( const FaceTxOffsets& ft, mat.txOffsets)
            {
                const int fid = ft.first;
                const cv::Vec3i& fvs = mat.faceVertexOrder.at(fid);
                const cv::Vec6f& txs = ft.second;
                std::cerr << "\tF_" << fid << ") with ordered vertices: "
                          << std::setw(2) << fvs[0]
                          << ", " << std::setw(2) << fvs[1]
                          << ", " << std::setw(2) << fvs[2] << "; TXs:  "
                          << cv::Vec2f( txs[0], txs[1])
                          << ", " << cv::Vec2f( txs[2], txs[3])
                          << ", " << cv::Vec2f( txs[4], txs[5]) << std::endl;
            }   // end foreach
        }   // end if
    }   // end foreach

    std::cerr << " --------------------------------------------------- " << std::endl;
    std::cerr << " C _vCounter =                " << std::setw(8) << _vCounter << std::endl;
    std::cerr << " C _faceCounter =             " << std::setw(8) << _faceCounter << std::endl;
    std::cerr << " C _edgeCounter =             " << std::setw(8) << _edgeCounter << std::endl;
    std::cerr << " --------------------------------------------------- " << std::endl;
    std::cerr << " _vtxIds.size() =             " << std::setw(8) << _vtxIds.size() << std::endl;
    std::cerr << " _verts.size() =              " << std::setw(8) << _verts.size() << std::endl;
    std::cerr << " _vtxToFaces.size() =         " << std::setw(8) << _vtxToFaces.size() << std::endl;
    std::cerr << " _vtxToEdges.size() =         " << std::setw(8) << _vtxToEdges.size() << std::endl;
    std::cerr << " _vtxConnections.size() =     " << std::setw(8) << _vtxConnections.size() << std::endl;
    std::cerr << " _vtxConnectionFaces.size() = " << std::setw(8) << _vtxConnectionFaces.size() << std::endl;
    std::cerr << " _verticesToUniqIdxs.size() = " << std::setw(8) << _verticesToUniqIdxs.size() << std::endl;
    std::cerr << " --------------------------------------------------- " << std::endl;
    std::cerr << " _faceIds.size() =            " << std::setw(8) << _faceIds.size() << std::endl;
    std::cerr << " _faces.size() =              " << std::setw(8) << _faces.size() << std::endl;
    std::cerr << " _faceMap.size() =            " << std::setw(8) << _faceMap.size() << std::endl;
    std::cerr << " _faceEdgeIdxs.size() =       " << std::setw(8) << _faceEdgeIdxs.size() << std::endl;
    std::cerr << " _faceMaterial.size() =       " << std::setw(8) << _faceMaterial.size() << std::endl;
    std::cerr << " --------------------------------------------------- " << std::endl;
    std::cerr << " _edgeIds.size() =            " << std::setw(8) << _edgeIds.size() << std::endl;
    std::cerr << " _edges.size() =              " << std::setw(8) << _edges.size() << std::endl;
    std::cerr << " _edgeMap.size() =            " << std::setw(8) << _edgeMap.size() << std::endl;
    std::cerr << " _edgesToFaces.size() =       " << std::setw(8) << _edgesToFaces.size() << std::endl;
    std::cerr << "=====================================================" << std::endl;
}   // end showDebug
