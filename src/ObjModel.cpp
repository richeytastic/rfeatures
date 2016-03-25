#include "ObjModel.h"
using RFeatures::ObjPoly;
using RFeatures::Edge;
using RFeatures::ObjModel;
#include <cmath>
#include <algorithm>
#include <iostream>
#include <cassert>
#include <algorithm>


// Two faces are the same if they share the same vertices (in whatever order)
bool ObjPoly::operator==( const ObjPoly& p) const
{
    bool same = false;
    if ( vindices[0] == p.vindices[0])
    {
        same = vindices[1] == p.vindices[1] && vindices[2] == p.vindices[2] ||
               vindices[2] == p.vindices[1] && vindices[1] == p.vindices[2];
    }   // end if
    else if ( vindices[0] == p.vindices[1])
    {
        same = vindices[1] == p.vindices[0] && vindices[2] == p.vindices[2] ||
               vindices[2] == p.vindices[0] && vindices[1] == p.vindices[2];
    }   // end else if
    else if ( vindices[0] == p.vindices[2])
    {
        same = vindices[1] == p.vindices[0] && vindices[2] == p.vindices[1] ||
               vindices[2] == p.vindices[0] && vindices[1] == p.vindices[1];
    }   // end else if
    return same;
}   // end operator==


// public
size_t ObjPoly::calcHash() const
{
    // Hash in order from min to max vertex IDs
    int mini = 0;
    if ( vindices[1] < vindices[0])
        mini = 1;
    if ( vindices[2] < vindices[1])
        mini = 2;
    size_t hval = (size_t)vindices[mini];
    const int minj = (mini + 1) % 3;
    const int mink = (mini + 2) % 3;
    if ( vindices[minj] < vindices[mink])
    {
        boost::hash_combine( hval, vindices[minj]);
        boost::hash_combine( hval, vindices[mink]);
    }   // end if
    else
    {
        boost::hash_combine( hval, vindices[mink]);
        boost::hash_combine( hval, vindices[minj]);
    }   // end else
    return hval;
}   // end calcHash


Edge::Edge() : v0(-1), v1(-1) {}
Edge::Edge( int u0, int u1) : v0(u0), v1(u1) {}

bool Edge::operator==( const Edge& e) const
{
    return (e.v0 == v0 && e.v1 == v1) || (e.v0 == v1 && e.v1 == v0);
}   // end operator==



cv::Vec3i makeFaceKeyFromUniqueVertexIds( int uv0, int uv1, int uv2)
{
    assert( uv0 != uv1);
    assert( uv1 != uv2);
    assert( uv0 != uv2);

    if ( uv0 > uv1)
    {
        int tmp = uv1;
        uv1 = uv0;
        uv0 = tmp;
    }   // end if
    // uv0 and uv1 in ascending order

    if ( uv2 < uv0)
    {
        int tmp = uv2;
        uv2 = uv1;
        uv1 = uv0;
        uv0 = tmp;
    }   // end if
    else if ( uv2 < uv1)
    {
        int tmp = uv2;
        uv2 = uv1;
        uv1 = tmp;
    }   // end else if
    // else uv2 > uv1 and uv2 doesn't need to be moved

    return cv::Vec3i( uv0, uv1, uv2);
}   // end makeFaceKeyFromUniqueVertexIds


// public
ObjModel::Ptr ObjModel::create( int fltPrc)
{
    return ObjModel::Ptr( new ObjModel(fltPrc));
}   // end create


// private
ObjModel::ObjModel( int fltPrc)
    :  _fltPrc( fltPrc), _uvCounter(0), _vCounter(0), _faceCounter(0), _edgeCounter(0)
{}   // end ctor


// public
ObjModel::~ObjModel()
{
    // Delete the faces
    typedef std::pair<int, const ObjPoly*> PPair;
    BOOST_FOREACH ( const PPair& face, _faces)
        delete face.second;
    BOOST_FOREACH ( const PPair& face, _ufaces)
        delete face.second;
    // Delete the edges
    typedef std::pair<int, const Edge*> EPair;
    BOOST_FOREACH ( const EPair& edge, _uedges)
        delete edge.second;
}   // end dtor


// public
int ObjModel::addVertex( const cv::Vec3f& v, const cv::Vec2f& uv)
{
    if ( cvIsNaN( cv::norm( v, cv::NORM_L2)))
        return -1;

    if ( cvIsNaN( uv[0]) || cvIsNaN( uv[1]))
        return -1;

    const cv::Vec3i key = toInt( v, _fltPrc);
    if ( !_verticesToIdxs.count( key))
        _verticesToIdxs[key] = _uvCounter++;
    const int ui = _verticesToIdxs[key];

    if ( !_uvtxIds.count(ui))
    {
        _uvtxIds.insert(ui);
        _uniqVerts[ui] = v;
        _uvfcounts[ui] = 0;
        _uvtxConnections[ui].clear();
        _uvtxConnectionFaces[ui].clear();
        _uvtxToFaces[ui].clear();
        _uvtxToEdges[ui].clear();
    }   // end if

    // Store
    const int vidx = _vCounter++;
    _vtxIds.insert(vidx);
    _vtxToUvtx[vidx] = ui;
    _vtxToTextures[vidx] = uv;
    _uvtxToVtx[ui].insert( vidx);    // Duplicate index store
    return vidx;
}   // end addVertex


// public
bool ObjModel::removeVertex( int vid)
{
    assert( getVertexIds().count(vid));
    assert( getFaceIds(vid).empty());
    const int ui = getUniqueVertexIdFromNonUnique(vid);

    const IntSet& cuvtxs = getConnectedUniqueVertices(ui);
    assert(cuvtxs.empty());
    if ( !cuvtxs.empty())
        return false;

    // Remove this vertex without removing the unique vertex since there may be duplicates.
    _vtxIds.erase(vid);
    _vtxToUvtx.erase(vid);
    _vtxToTextures.erase(vid);
    _uvtxToVtx[ui].erase(vid);

    // Delete the unique vertex too if no more vertices
    if ( _uvtxToVtx[ui].empty())
    {
        const cv::Vec3i key = toInt( _uniqVerts[ui], _fltPrc);
        _verticesToIdxs.erase(key);

        _uvtxIds.erase(ui);
        _uniqVerts.erase(ui);
        _uvtxCurvature.erase(ui);
        _uvtxToVtx.erase(ui);
        _uvfcounts.erase(ui);

        // Copy out the connected unique vertex IDs to reevaluate their flatness and boundary properties:
        const IntSet cuvs = cuvtxs;

        _uvtxConnections.erase(ui);
        _uvtxConnectionFaces.erase(ui);

        _uvtxToFaces.erase(ui);
        _uvtxToEdges.erase(ui);
    }   // end if

    return true;
}   // end removeVertex


// public
bool ObjModel::removeUniqueVertex( int ui)
{
    const IntSet& cuvtxs = getConnectedUniqueVertices(ui);
    assert(cuvtxs.empty());
    if ( !cuvtxs.empty())
        return false;

    assert( getUniqueVertexIds().count(ui));
    assert( getFaceIdsFromUniqueVertex(ui).empty());
    const IntSet vids = _uvtxToVtx[ui]; // Copy out because _uvtxToVtx[ui] will be changed
    BOOST_FOREACH ( const int& vid, vids)
        removeVertex(vid);
    return true;
}   // end removeUniqueVertex


// public
bool ObjModel::isLonely( int uvid) const
{
    return getConnectedUniqueVertices(uvid).empty();
}   // end isLonely


// public
bool ObjModel::isFlat( int uvid) const
{
    const IntSet& cuvtx = getConnectedUniqueVertices( uvid);
    BOOST_FOREACH ( const int& ui, cuvtx)
    {
        if ( getNumSharedFaces( uvid, ui) > 2)
            return false;
    }   // end foreach
    return true;
}   // end isFlat


// public
bool ObjModel::isBoundary( int uvid) const
{
    const IntSet& cuvtx = getConnectedUniqueVertices( uvid);
    BOOST_FOREACH ( const int& ui, cuvtx)
    {
        if ( getNumSharedFaces( uvid, ui) == 1)
            return true;
    }   // end foreach
    return false;
}   // end isBoundary


// public
// A vertex x is a "flat boundary" if there are exactly two connected vertices
// that share a single face (each) with the vertex x.
bool ObjModel::isFlatBoundary( int uvid) const
{
    int bcount = 0;
    const IntSet& cuvtx = getConnectedUniqueVertices( uvid);
    BOOST_FOREACH ( const int& ui, cuvtx)
    {
        if ( getNumSharedFaces( uvid, ui) == 1)
        {
            bcount++;
            if ( bcount > 2)    // No need to continue since now known to be false
                break;
        }   // end if
    }   // end foreach
    return bcount == 2;
}   // end isFlatBoundary


// public
// A vertex x is a "junction" if is has greater than two connected vertices
// that share a single face (each) with the vertex x.
bool ObjModel::isJunction( int uvid) const
{
    int bcount = 0;
    const IntSet& cuvtx = getConnectedUniqueVertices( uvid);
    BOOST_FOREACH ( const int& ui, cuvtx)
    {
        if ( getNumSharedFaces( uvid, ui) == 1)
        {
            bcount++;
            if ( bcount > 2)    // No need to continue since now known to be true
                return true;
        }   // end if
    }   // end foreach
    return false;
}   // end isJunction


// public
bool ObjModel::isTip( int uvid) const
{
    return getUniqueVertexFaceCount(uvid) == 1;
}   // end isTip


// public
int ObjModel::getUniqueVertexFaceCount( int uvid) const
{
    assert( _uvfcounts.count(uvid));
    return _uvfcounts.at(uvid);
}   // end getUniqueVertexFaceCount


// Provide v2 such that it makes a positive angle at v0 with the vector v1-v0
cv::Vec3f computeClockwiseNormal( const cv::Vec3f& v0, const cv::Vec3f& v1, const cv::Vec3f& v2)
{
    const cv::Vec3f d0 = v1 - v0;
    const cv::Vec3f d1 = v2 - v0;
    const cv::Vec3f fn = d0.cross(d1);
    cv::Vec3f outVec;
    cv::normalize( fn, outVec);  // L2 norm
    return outVec;
}   // end computeClockwiseNormal


// public
cv::Vec3f ObjModel::getNormal( int uvid) const
{
    cv::Vec3f nrm(0,0,0);
    const IntSet& fset = getFaceIdsFromUniqueVertex( uvid);
    BOOST_FOREACH ( const int& fid, fset)
        nrm += getFace(fid).nrmVec;
    cv::Vec3f onrm;
    cv::normalize( nrm, onrm);
    return onrm;
}   // end getNormal


// private
void ObjModel::setEdge( int faceIdx, int u0, int u1)
{
    int eidx = 0;
    const Edge* edge = new Edge( u0, u1);

    // Get the edges already associated with each of the vertices.
    // Only add the edges that aren't already included.
    if ( _uvtxToEdges.count(u0))
    {
        // Get the indices into _uedges to compare
        const IntSet& eindices = _uvtxToEdges.at(u0);
        BOOST_FOREACH ( const int& ei, eindices)
        {
            if ( *_uedges[ei] == *edge)    // Was the edge already added previously?
            {
                delete edge;
                edge = NULL;
                eidx = ei;
                break;
            }   // end if
        }   // end foreach
    }   // end if

    // Add a new edge
    if ( edge)
    {
        eidx = _edgeCounter++;
        _uedges[eidx] = edge;  // Add the edge object
        _edgeIdxs.insert(eidx);
        _uvtxToEdges[u0].insert(eidx);
        _uvtxToEdges[u1].insert(eidx);
    }   // end else

    // Set the face/edge relation
    _faceEdgeIdxs[faceIdx].insert( eidx);
    _edgesToFaces[eidx].insert(faceIdx);
}   // end setEdge


// private
void ObjModel::removeFaceEdges( int faceIdx)
{
    assert( _faceEdgeIdxs.count(faceIdx));
    const IntSet edgeIds = _faceEdgeIdxs.at(faceIdx);   // Copy out
    BOOST_FOREACH ( const int& eidx, edgeIds)
    {
        _faceEdgeIdxs[faceIdx].erase(eidx);
        _edgesToFaces[eidx].erase(faceIdx);

        if ( _edgesToFaces.at(eidx).empty())
        {   // An edge is only removed if it has no more faces in common
            const Edge& edge = getEdge(eidx);
            _uvtxToEdges[edge.v0].erase(eidx);
            _uvtxToEdges[edge.v1].erase(eidx);
            _edgeIdxs.erase(eidx);
            delete _uedges.at(eidx);
            _uedges.erase(eidx);
            _edgesToFaces.erase(eidx);
        }   // end if
    }   // end foreach
    _faceEdgeIdxs.erase(faceIdx);
}   // end removeFaceEdges


// public
int ObjModel::setFace( const int* vtxs)
{
    return setFace( vtxs[0], vtxs[1], vtxs[2]);
}   // end setFace


// public
int ObjModel::setFace( int v0, int v1, int v2)
{
    ObjPoly *uface = new ObjPoly;
    const int u0 = getUniqueVertexIdFromNonUnique(v0);
    const int u1 = getUniqueVertexIdFromNonUnique(v1);
    const int u2 = getUniqueVertexIdFromNonUnique(v2);
    assert( _uvtxIds.count(u0));
    assert( _uvtxIds.count(u1));
    assert( _uvtxIds.count(u2));
    uface->vindices[0] = u0;
    uface->vindices[1] = u1;
    uface->vindices[2] = u2;

    // If this face is already present, don't add!
    const size_t fhash = uface->calcHash();
    if ( _ufaceHashes.count(fhash))
    {
        // Check each face in more detail
        const std::list<const ObjPoly*>& plist = _ufaceHashes.at(fhash);
        BOOST_FOREACH ( const ObjPoly* p, plist)
        {
            if ( *p == *uface)
            {
                delete uface;
                return -1;
            }   // end if
        }   // end foreach
    }   // end if

    ObjPoly *face = new ObjPoly;
    assert( _vtxIds.count(v0));
    assert( _vtxIds.count(v1));
    assert( _vtxIds.count(v2));
    face->vindices[0] = v0;
    face->vindices[1] = v1;
    face->vindices[2] = v2;

    const int faceIdx = _faceCounter++;

    _uvfcounts[u0]++;
    _uvfcounts[u1]++;
    _uvfcounts[u2]++;

    _uvtxToFaces[u0].insert(faceIdx);
    _uvtxToFaces[u1].insert(faceIdx);
    _uvtxToFaces[u2].insert(faceIdx);

    // Connect each vertex to its neighbours on the face (the
    // connection may already exist but okay to insert because using a set).
    _uvtxConnections[u0].insert(u1);
    _uvtxConnections[u0].insert(u2);
    _uvtxConnections[u1].insert(u0);
    _uvtxConnections[u1].insert(u2);
    _uvtxConnections[u2].insert(u0);
    _uvtxConnections[u2].insert(u1);

    // Increment the number of faces each connected vertex is connected to
    _uvtxConnectionFaces[u0][u1].insert(faceIdx);
    _uvtxConnectionFaces[u0][u2].insert(faceIdx);
    _uvtxConnectionFaces[u1][u0].insert(faceIdx);
    _uvtxConnectionFaces[u1][u2].insert(faceIdx);
    _uvtxConnectionFaces[u2][u0].insert(faceIdx);
    _uvtxConnectionFaces[u2][u1].insert(faceIdx);

    // Add three edges
    setEdge( faceIdx, u0, u1);
    setEdge( faceIdx, u1, u2);
    setEdge( faceIdx, u2, u0);

    // Set the face normal - assumes that face vertices are ordered clockwise
    uface->nrmVec = computeClockwiseNormal( getVertex(v0), getVertex(v1), getVertex(v2));
    face->nrmVec = uface->nrmVec;
    _faces[faceIdx] = face;
    _ufaces[faceIdx] = uface;
    _ufaceHashes[fhash].push_back( uface);
    _faceIdxs.insert(faceIdx);

    return faceIdx;
}   // end setFace


// public
bool ObjModel::unsetFace( int faceIdx)
{
    assert( _faces.count(faceIdx));

    const ObjPoly& uface = getUniqueVertexFace(faceIdx);
    const int u0 = uface.vindices[0];
    const int u1 = uface.vindices[1];
    const int u2 = uface.vindices[2];
    assert( _uvtxIds.count(u0));
    assert( _uvtxIds.count(u1));
    assert( _uvtxIds.count(u2));

    const ObjPoly& face = getFace(faceIdx);
    const int v0 = face.vindices[0];
    const int v1 = face.vindices[1];
    const int v2 = face.vindices[2];
    assert( _vtxIds.count(v0));
    assert( _vtxIds.count(v1));
    assert( _vtxIds.count(v2));

    _uvfcounts[u0]--;
    _uvfcounts[u1]--;
    _uvfcounts[u2]--;

    _uvtxToFaces[u0].erase(faceIdx);
    _uvtxToFaces[u1].erase(faceIdx);
    _uvtxToFaces[u2].erase(faceIdx);

    _uvtxConnectionFaces[u0][u1].erase(faceIdx);
    _uvtxConnectionFaces[u0][u2].erase(faceIdx);
    _uvtxConnectionFaces[u1][u0].erase(faceIdx);
    _uvtxConnectionFaces[u1][u2].erase(faceIdx);
    _uvtxConnectionFaces[u2][u0].erase(faceIdx);
    _uvtxConnectionFaces[u2][u1].erase(faceIdx);

    // if the connection between vertices shares no more faces,
    // remove the entry from uvtxConnections
    if ( _uvtxConnectionFaces[u0][u1].empty())
    {
        _uvtxConnections[u0].erase(u1);
        _uvtxConnectionFaces[u0].erase(u1);
        _uvtxConnections[u1].erase(u0);
        _uvtxConnectionFaces[u1].erase(u0);
    }   // end if
    if ( _uvtxConnectionFaces[u1][u2].empty())
    {
        _uvtxConnections[u1].erase(u2);
        _uvtxConnectionFaces[u1].erase(u2);
        _uvtxConnections[u2].erase(u1);
        _uvtxConnectionFaces[u2].erase(u1);
    }   // end if
    if ( _uvtxConnectionFaces[u2][u0].empty())
    {
        _uvtxConnections[u2].erase(u0);
        _uvtxConnectionFaces[u2].erase(u0);
        _uvtxConnections[u0].erase(u2);
        _uvtxConnectionFaces[u0].erase(u2);
    }   // end if

    removeFaceEdges( faceIdx);

    _faceCurvature.erase(faceIdx);

    // Remove the entry from the hash list
    const size_t fhash = uface.calcHash();
    std::list<const ObjPoly*>& plist = _ufaceHashes.at(fhash);
    for ( std::list<const ObjPoly*>::iterator it = plist.begin(); it != plist.end(); ++it)
    {
        if ( **it == uface)
        {
            plist.erase(it);
            break;
        }   // end if
    }   // end for

    delete _ufaces.at(faceIdx);
    delete _faces.at(faceIdx);
    _faces.erase(faceIdx);
    _ufaces.erase(faceIdx);
    _faceIdxs.erase(faceIdx);

    return true;
}   // end unsetFace


// public
const ObjPoly& ObjModel::getFace( int faceId) const
{
    assert( _faceIdxs.count(faceId));
    assert( _faces.count(faceId));
    assert( _ufaces.count(faceId));
    assert( _faceEdgeIdxs.count(faceId));
    return *_faces.at(faceId);
}   // end getFace


// public
const ObjPoly& ObjModel::getUniqueVertexFace( int faceId) const
{
    assert( _faceIdxs.count(faceId));
    assert( _faces.count(faceId));
    assert( _ufaces.count(faceId));
    assert( _faceEdgeIdxs.count(faceId));
    return *_ufaces.at(faceId);
}   // end getUniqueVertexFace


// public
int ObjModel::getFaceConnectivityMetric( int faceId) const
{
    if ( !_faceIdxs.count(faceId))
        return -1;

    const ObjPoly& face = getUniqueVertexFace( faceId);
    int csum = -3;
    csum += getFaceIdsFromUniqueVertex( face.vindices[0]).size();
    csum += getFaceIdsFromUniqueVertex( face.vindices[1]).size();
    csum += getFaceIdsFromUniqueVertex( face.vindices[2]).size();
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
const IntSet& ObjModel::getFaceIds( int vertexId) const
{
    return getFaceIdsFromUniqueVertex( _vtxToUvtx.at(vertexId));
}   // end getFaceIds


// public
const Edge& ObjModel::getEdge( int edgeId) const
{
    assert( _uedges.count(edgeId));
    return *_uedges.at(edgeId);
}   // end getEdge


// public
const IntSet& ObjModel::getConnectedUniqueVertices( int ui) const
{
    assert( _uvtxIds.count(ui));
    assert( _uvtxConnections.count(ui));
    return _uvtxConnections.at(ui);
}   // end getConnectedUniqueVertices


// public
int ObjModel::getNumSharedFaces( int ui, int uj) const
{
    if ( !_uvtxConnectionFaces.count(ui) || !_uvtxConnectionFaces.count(uj))
        return -1;

    const boost::unordered_map<int, IntSet>& uicf = _uvtxConnectionFaces.at(ui);
    if ( !uicf.count(uj))
        return 0;

#ifndef NDEBUG
    const boost::unordered_map<int, IntSet>& ujcf = _uvtxConnectionFaces.at(uj);
    assert( uicf.at(uj) == ujcf.at(ui));
#endif
    return uicf.at(uj).size();
}   // end getNumSharedFaces


// public
const IntSet& ObjModel::getSharedFaces( int ui, int uj) const
{
    assert( _uvtxConnectionFaces.count(ui) && _uvtxConnectionFaces.count(uj));
    const boost::unordered_map<int, IntSet>& uicf = _uvtxConnectionFaces.at(ui);
    assert( uicf.count(uj));
    return uicf.at(uj);
}   // end getSharedFaces


// public
const IntSet& ObjModel::getFaceIdsFromUniqueVertex( int ui) const
{
    assert( _uvtxIds.count(ui));
    assert( _uvtxToFaces.count(ui));
    return _uvtxToFaces.at(ui);
}   // end getFaceIdsFromUniqueVertex


// public
const IntSet& ObjModel::lookupVertexIndices( const cv::Vec3f& v) const
{
    const int uvid = lookupUniqueVertexIndex( v);
    assert( uvid >= 0);
    return lookupTextureIndices(uvid);
}   // end lookupVertexIndices


// public
int ObjModel::lookupUniqueVertexIndex( const cv::Vec3f& v) const
{
    const cv::Vec3i key = toInt( v, _fltPrc);
    if ( !_verticesToIdxs.count( key))
        return -1;
    return _verticesToIdxs.at(key);   // Get the unique vertex ID
}   // end lookupUniqueVertexIndex


// public
const IntSet& ObjModel::lookupTextureIndices( int ui) const
{
    assert( _uvtxIds.count(ui));
    assert( _uvtxToVtx.count(ui));
    return _uvtxToVtx.at(ui);
}   // end lookupTextureIndices


// private
void ObjModel::addFaceFromModel( const ObjModel* model, const ObjPoly& face)
{
    int vids[ObjPoly::NVTX];
    // Add the vertices for the face
    for ( int i = 0; i < ObjPoly::NVTX; ++i)
    {
        const int vid = face.vindices[i];
        vids[i] = addVertex( model->getVertex(vid), model->getTextureOffset(vid));
    }   // end for
    setFace( vids);
}   // end addFace


ObjModel::Ptr ObjModel::createFromUniqueVertices( const std::vector<int>& uvids) const
{
    // Create a new object from the selected data
    ObjModel::Ptr obj = ObjModel::create();
    obj->setTexture( this->getTexture());

    IntSet facesToAdd;   // Identifies the faces to add
    IntSet facesNotToAdd;    // Faces identified as not to be added
    boost::unordered_map<int, int> oldVtxsToNewVtxs;    // Maps vertex ID from _objModel to newly added vertex IDs in obj
    IntSet uvidsSet( uvids.begin(), uvids.end());
    BOOST_FOREACH ( const int& uvid, uvidsSet)
    {
        // uvid must be present in the original model
        const IntSet& vindices = lookupTextureIndices(uvid);

        // Set the original vertices/textures in the new model. All the original vertices will
        // be added even though some of them may not be set as part of a triangular face of this model.
        BOOST_FOREACH ( const int& vid, vindices)
        {
            const cv::Vec3f& v = this->getVertex(vid);
            const cv::Vec2f& uv = this->getTextureOffset(vid);
            const int addIdx = obj->addVertex( v, uv);
            assert( obj->getVertexIds().count(addIdx));
            oldVtxsToNewVtxs[vid] = addIdx;
        }   // end for

        const IntSet& faceIds = getFaceIdsFromUniqueVertex(uvid);
        // For each face this unique vertex is involved with, we count the face to be set only if
        // the unique vertices for all of the face's corners are present in uvids, otherwise it is
        // counted as a face that should not be added (so the face vertex check can be skipped if the
        // face ID is encountered again).
        BOOST_FOREACH ( const int& fid, faceIds)
        {
            // If this face ID has already been added, or already identified as one that should not be added, it is skipped.
            if ( facesToAdd.count(fid) || facesNotToAdd.count(fid))
                continue;
            const ObjPoly& uface = getUniqueVertexFace(fid);
            if ( uvidsSet.count( uface.vindices[0]) && uvidsSet.count( uface.vindices[1]) && uvidsSet.count( uface.vindices[2]))
                facesToAdd.insert( fid);
            else
                facesNotToAdd.insert( fid);
        }   // end foreach
    }   // end foreach

    // Finally, set the faces
    int fvids[3];
    BOOST_FOREACH ( const int& i, facesToAdd)
    {
        const RFeatures::ObjPoly& face = getFace(i); // Faces with original vertex IDs

        const int v0 = face.vindices[0];
        const int v1 = face.vindices[1];
        const int v2 = face.vindices[2];
        assert( oldVtxsToNewVtxs.count(v0));
        assert( oldVtxsToNewVtxs.count(v1));
        assert( oldVtxsToNewVtxs.count(v2));
        fvids[0] = oldVtxsToNewVtxs[v0];
        fvids[1] = oldVtxsToNewVtxs[v1];
        fvids[2] = oldVtxsToNewVtxs[v2];
        obj->setFace( fvids);
    }   // end foreach

    return obj;
}   // end createFromUniqueVertices


// private
double ObjModel::calcDeterminant( const ObjPoly& uface) const
{
    const cv::Vec3f n0 = getNormal( uface.vindices[0]);
    const cv::Vec3f n1 = getNormal( uface.vindices[1]);
    const cv::Vec3f n2 = getNormal( uface.vindices[2]);
    return n2.dot(n0.cross(n1)); // Calculate determinant as the scalar triple product
}   // end calcDeterminant


// private
void ObjModel::generateFaceCurvature( int fid)
{
    assert( _faceIdxs.count(fid));
    _faceCurvature[fid] = calcDeterminant( getUniqueVertexFace(fid));
}   // end generateFaceCurvature


// private
void ObjModel::generateUniqueVertexCurvature( int uvid)
{
    assert( _uvtxIds.count(uvid));
    const IntSet& fids = getFaceIdsFromUniqueVertex( uvid);
    double curv = 0;
    BOOST_FOREACH ( const int& fid, fids)
        curv += getFaceCurvature(fid);
    _uvtxCurvature[uvid] = curv / fids.size();
}   // end generateUniqueVertexCurvature


// public
double ObjModel::getFaceCurvature( int fid)
{
    assert( _faceIdxs.count(fid));
    if ( !_faceCurvature.count(fid))
        generateFaceCurvature(fid);
    return _faceCurvature.at(fid);
}   // end getFaceCurvature


// public
double ObjModel::getUniqueVertexCurvature( int uvid)
{
    assert( _uvtxIds.count(uvid));
    if ( !_uvtxCurvature.count(uvid))
        generateUniqueVertexCurvature(uvid);
    return _uvtxCurvature.at(uvid);
}   // end getUniqueVertexCurvature
