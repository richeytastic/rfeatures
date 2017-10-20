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
    fvindices[0] = v0;
    fvindices[1] = v1;
    fvindices[2] = v2;
}   // end ctor


// Two ObjPolys are the same if they share the same vertices
bool ObjPoly::operator==( const ObjPoly& p) const
{
    return (fvindices[0] == p.fvindices[0]) && (fvindices[1] == p.fvindices[1]) && (fvindices[2] == p.fvindices[2]);
}   // end operator==


// public
bool ObjPoly::getOpposite( int vid, int& other0, int& other1) const
{
    bool found = true;

    if ( vid == fvindices[0])
    {
        other0 = fvindices[1];
        other1 = fvindices[2];
    }   // end if
    else if ( vid == fvindices[1])
    {
        other0 = fvindices[0];
        other1 = fvindices[2];
    }   // end else if
    else if ( vid == fvindices[2])
    {
        other0 = fvindices[0];
        other1 = fvindices[1];
    }   // end else if
    else
        found = false;

    return found;
}   // end getOpposite


// public
int ObjPoly::getOpposite( int v0, int v1) const
{
    int vn = fvindices[0];
    if ( vn == v0 || vn == v1)
    {
        vn = fvindices[1];
        if ( vn == v0 || vn == v1)
            vn = fvindices[2];
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
    boost::hash_combine( seed, u.fvindices[0]);
    boost::hash_combine( seed, u.fvindices[1]);
    boost::hash_combine( seed, u.fvindices[2]);
    return seed;
}   // end operator()


size_t RFeatures::HashEdge::operator()( const Edge& u) const
{
    size_t seed = 0;
    boost::hash_combine( seed, u.v0);
    boost::hash_combine( seed, u.v1);
    return seed;
}   // end operator()


struct ObjModel::Material
{
    std::vector<cv::Mat> ambient;
    std::vector<cv::Mat> diffuse;
    std::vector<cv::Mat> specular;

    IntSet uvIds;
    boost::unordered_map<int, cv::Vec2f> uvs;               // UV IDs to UVs 
    boost::unordered_map<int, IntSet> uvFaceRefs;           // UV IDs --> Face IDs 

    IntSet faceIds;
    boost::unordered_map<int, cv::Vec3i> faceVertexOrder;   // Face IDs --> Vertex IDs
    boost::unordered_map<int, cv::Vec3i> faceUVOrder;       // Face IDs --> UV IDs

    Key2LToIntMap _uvToUniqIdxs;              // How UVs map to the keys of uvs
    int _uvCounter;

    Material() : _uvCounter(0) {}

    int lookupUVIndex( const cv::Vec2f& uv, int fltPrc) const
    {
        Key2L key = toKey( uv, fltPrc);
        return _uvToUniqIdxs.count( key) > 0 ? _uvToUniqIdxs.at(key) : -1;
    }   // end lookupUVIndex

    // Update existing UV position
    void updateUV( int uvID, const cv::Vec2f& newPos, int fltPrc)
    {
        assert( uvIds.count(uvID) > 0);
        Key2L key = toKey( uvs.at(uvID), fltPrc);
        _uvToUniqIdxs.erase(key);
        key = toKey( newPos, fltPrc);
        _uvToUniqIdxs[key] = uvID;
        uvs[uvID] = newPos;
    }   // end updateUV

private:
    Material( const Material&); // No copy
    void operator=( const Material&); // No copy
};  // end class



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
        (*iimap)[vid] = nm->addVertex( omc->vtx(vid));
    }   // end foreach

    const IntSet& matIds = omc->getMaterialIds();
    BOOST_FOREACH ( int mid, matIds)
    {
        const int mid2 = nm->addMaterial();

        if ( shareMaterials)
        {
            BOOST_FOREACH ( const cv::Mat& img, omc->getMaterialAmbient(mid))
                nm->addMaterialAmbient( mid2, img);
            BOOST_FOREACH ( const cv::Mat& img, omc->getMaterialDiffuse(mid))
                nm->addMaterialDiffuse( mid2, img);
            BOOST_FOREACH ( const cv::Mat& img, omc->getMaterialSpecular(mid))
                nm->addMaterialSpecular( mid2, img);
        }   // end if
        else
        {
            BOOST_FOREACH ( const cv::Mat& img, omc->getMaterialAmbient(mid))
                nm->addMaterialAmbient( mid2, img.clone());
            BOOST_FOREACH ( const cv::Mat& img, omc->getMaterialDiffuse(mid))
                nm->addMaterialDiffuse( mid2, img.clone());
            BOOST_FOREACH ( const cv::Mat& img, omc->getMaterialSpecular(mid))
                nm->addMaterialSpecular( mid2, img.clone());
        }   // end else
    }   // end for

    // Copy over the faces from the old model ensuring that the old vertex IDs from
    // the old model map to the new vertex IDs in the new model.
    const IntSet& fids = omc->getFaceIds();
    BOOST_FOREACH ( int fid, fids)
    {
        const int* vids = omc->getFaceVertices(fid);
        const int newFaceId = nm->setFace( iimap->at(vids[0]), iimap->at(vids[1]), iimap->at(vids[2]));
        const int matId = omc->getFaceMaterialId( fid);
        if ( matId >= 0)
        {
            const int* uvids = omc->getFaceUVs(fid);
            nm->setOrderedFaceUVs( matId, newFaceId, iimap->at(vids[0]), omc->uv(matId, uvids[0]),
                                                     iimap->at(vids[1]), omc->uv(matId, uvids[1]),
                                                     iimap->at(vids[2]), omc->uv(matId, uvids[2]));
        }   // end if
    }   // end foreach

    delete iimap;
    return nm;
}   // end copy


class ObjModel::Deleter
{ public:
    void operator()( const ObjModel* model) { delete model;}
};  // end class


// public
ObjModel::Ptr ObjModel::clone( bool shareMats)
{
    return ObjModel::copy( Ptr(this, Deleter()), shareMats);
}   // end clone


// public static
ObjModel::Ptr ObjModel::create( int fltPrc)
{
    return Ptr( new ObjModel(fltPrc), Deleter());
}   // end create


// private
ObjModel::ObjModel( int fltPrc)
    :  _fltPrc( fltPrc), _vCounter(0), _faceCounter(0), _edgeCounter(0), _materialCounter(0)
{}   // end ctor


// private
ObjModel::~ObjModel()
{
    removeAllMaterials();
}   // end dtor


// public
int ObjModel::getFaceMaterialId( int fid) const
{
    assert( _faceIds.count(fid));
    if ( !_faceMaterial.count(fid)) // No material for this face
        return -1;
    return _faceMaterial.at(fid);
}   // end getFaceMaterialId


// public
const IntSet& ObjModel::getMaterialFaceIds( int mid) const
{
    static const IntSet EMPTY_INT_SET;
    if ( _materials.count(mid) == 0)
        return EMPTY_INT_SET;
    return _materials.at(mid)->faceIds;
}   // end getMaterialFaceIds


// public
int ObjModel::addMaterial()
{
    const int mid = _materialCounter++;
    _materialIds.insert(mid);
    _materials[mid] = new Material();
    return mid;
}   // end addMaterial


// public
bool ObjModel::removeMaterial( int materialID)
{
    if ( _materials.count(materialID) == 0)
        return false;
    _materialIds.erase(materialID);
    delete _materials.at(materialID);
    _materials.erase(materialID);
    return true;
}   // end removeMaterial


// public
bool ObjModel::removeAllMaterials()
{
    if ( _materialIds.empty())
        return false;
    const IntSet mids = getMaterialIds();   // Copy out
    BOOST_FOREACH( int mid, mids)
        removeMaterial(mid);
    return true;
}   // end removeAllMaterials


// Calculate a new UV coordinate from an old UV.
void calcNewUV( cv::Vec2f& uv, int nrows, int ncols, const std::vector<int>& scols, int i)
{
    // v is unchanged, only u affected
    const float oldWidth = (i == scols.size()-1) ? ncols - scols[i] : scols[i+1] - scols[i];
    uv[0] = (scols[i] + (uv[0] * oldWidth))/ncols;
}   // end calcNewUV


// public
size_t ObjModel::mergeMaterials()
{
    const IntSet mids = getMaterialIds();   // Copied out because changing
    const int nmats = (int)mids.size();
    if ( nmats <= 1)
        return nmats;

    // For each image type (ambient, diffuse, and specular), concatenate the
    // images across all of the materials into a single texture image.
    std::vector<cv::Mat> aimgs; // Ambient images from all materials
    std::vector<cv::Mat> dimgs; // Diffuse images from all materials
    std::vector<cv::Mat> simgs; // Specular images from all materials
    std::vector<int> midSeq;    // Repeatable sequence of material IDs
    BOOST_FOREACH ( int mid, mids)
    {
        midSeq.push_back(mid);
        const std::vector<cv::Mat>& ams = getMaterialAmbient(mid);
        const std::vector<cv::Mat>& dms = getMaterialDiffuse(mid);
        const std::vector<cv::Mat>& sms = getMaterialSpecular(mid);
        if ( !ams.empty())
            aimgs.push_back(ams[0]);
        if ( !dms.empty())
            dimgs.push_back(dms[0]);
        if ( !sms.empty())
            simgs.push_back(sms[0]);
    }   // end for

    std::vector<int> scols; // The starting columns for the concatenated texture images
    const cv::Mat aimg = RFeatures::concatHorizontalMax( aimgs, &scols);
    const cv::Mat dimg = RFeatures::concatHorizontalMax( dimgs, aimg.empty() ? &scols : NULL);
    const cv::Mat simg = RFeatures::concatHorizontalMax( simgs, dimg.empty() ? &scols : NULL);
    const int nrows = std::max(aimg.rows, std::max( dimg.rows, simg.rows)); // Number of rows of concatenated image
    const int ncols = std::max(aimg.cols, std::max( dimg.cols, simg.cols)); // Number of columns of concatenated image

    const int mmid = addMaterial(); // Create the new "merge" material
    addMaterialAmbient( mmid, aimg);
    addMaterialDiffuse( mmid, dimg);
    addMaterialSpecular( mmid, simg);

    int mid;
    for ( int i = 0; i < nmats; ++i)
    {
        mid = midSeq[i];

        // Map all the faces from material m to mmat
        const IntSet& fids = getMaterialFaceIds( mid);
        BOOST_FOREACH ( int fid, fids)
        {
            // Get and set the new texture offsets for the face based on
            // the horizontal concatentation of the texture images.
            const int* vidxs = getFaceVertices(fid);
            const int* uvidxs = getFaceUVs(fid);

            cv::Vec2f uv0 = uv(mid, uvidxs[0]);
            cv::Vec2f uv1 = uv(mid, uvidxs[1]);
            cv::Vec2f uv2 = uv(mid, uvidxs[2]);

            // Calculate new offsets
            calcNewUV( uv0, nrows, ncols, scols, i);
            calcNewUV( uv1, nrows, ncols, scols, i);
            calcNewUV( uv2, nrows, ncols, scols, i);

            // Set in the merged material
            setOrderedFaceUVs( mmid, fid, vidxs[0], uv0, vidxs[1], uv1, vidxs[2], uv2);
        }   // end foreach

        removeMaterial( mid);   // Remove the old material
    }   // end for

    return nmats;   // The number of materials that were merged.
}   // end mergeMaterials


// public
bool ObjModel::addMaterialAmbient( int materialID, const cv::Mat& m, size_t maxd)
{
    if ( _materials.count( materialID) == 0 || m.empty())
        return false;
    _materials[materialID]->ambient.push_back( RFeatures::shrinkMax( m, maxd));
    return true;
}   // end addMaterialAmbient


// public
bool ObjModel::addMaterialDiffuse( int materialID, const cv::Mat& m, size_t maxd)
{
    if ( _materials.count( materialID) == 0 || m.empty())
        return false;
    _materials[materialID]->diffuse.push_back( RFeatures::shrinkMax( m, maxd));
    return true;
}   // end addMaterialDiffuse


// public
bool ObjModel::addMaterialSpecular( int materialID, const cv::Mat& m, size_t maxd)
{
    if ( _materials.count( materialID) == 0 || m.empty())
        return false;
    _materials[materialID]->specular.push_back( RFeatures::shrinkMax( m, maxd));
    return true;
}   // end addMaterialSpecular


// public
const std::vector<cv::Mat>& ObjModel::getMaterialAmbient( int mid) const
{
    assert( _materials.count(mid) > 0);
    return _materials.at(mid)->ambient;
}   // end getMaterialAmbient


// public
const std::vector<cv::Mat>& ObjModel::getMaterialDiffuse( int mid) const
{
    assert( _materials.count(mid) > 0);
    return _materials.at(mid)->diffuse;
}   // end getMaterialDiffuse


// public
const std::vector<cv::Mat>& ObjModel::getMaterialSpecular( int mid) const
{
    assert( _materials.count(mid) > 0);
    return _materials.at(mid)->specular;
}   // end getMaterialSpecular


// private
void ObjModel::removeFaceUVs( int matid, int fid)
{
    assert( getMaterialIds().count( matid));
    Material& m = *_materials.at( matid);
    assert( m.faceVertexOrder.count(fid) == 1);

    // Erase this face ID from the uvFaceRefs. If any of the UVs are
    // no longer referenced by any faces, remove them from the material.
    int uvi;
    const cv::Vec3i& uvis = m.faceUVOrder.at(fid);
    for ( int i = 0; i < 3; ++i)
    {
        uvi = uvis[i];
        // Check for presence of m.uvFaceRefs.at(uvi) because the entry may
        // have been deleted on a previous iteration through this loop.
        if ( m.uvFaceRefs.count(uvi) > 0)
        {
            m.uvFaceRefs.at(uvi).erase(fid);

            // If no more face references to this UV, we can remove the UV itself.
            if ( m.uvFaceRefs.at(uvi).empty())
            {
                m.uvFaceRefs.erase(uvi);
                Key2L key = toKey( m.uvs.at(uvi), _fltPrc);
                m._uvToUniqIdxs.erase(key);
                m.uvs.erase(uvi);
                m.uvIds.erase(uvi);
            }   // end if
        }   // end if
    }   // end for

    m.faceVertexOrder.erase(fid);
    m.faceUVOrder.erase(fid);
    m.faceIds.erase(fid);
    _faceMaterial.erase(fid);
}   // end removeFaceUVs


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
        _verts[vi] = cv::Vec3f( float(x), float(y), float(z));
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
cv::Vec2f ObjModel::calcTextureCoord( int fidx, const cv::Vec3f& v) const
{
    const int matId = getFaceMaterialId( fidx);
    if ( matId < 0)
        return cv::Vec2f(-1,-1);

    // Create the new texture offset - assumes v is in the plane of the polygon.
    const int* vidxs = getFaceVertices(fidx);
    const cv::Vec3f& v0 = vtx(vidxs[0]);
    const cv::Vec3f& v2 = vtx(vidxs[2]);

    const cv::Vec3f v02 = v0 - v2;
    const cv::Vec3f vv2 = v - v2;
    const double v02len = cv::norm(v02);    // All vectors scaled to be in proportion of this length

    const double h = cv::norm(vv2)/v02len;       // Length of hypotenuse (scaled)
    const double a = vv2.dot(v02)/pow(v02len,2); // Projected length (scaled) of v - v2 along v0 - v2
    const double o = sqrt( h*h - a*a);           // Length of opposite side (scaled)

    const int* uvs = getFaceUVs( fidx);
    const cv::Vec2f& uv0 = uv( matId, uvs[0]);
    const cv::Vec2f& uv2 = uv( matId, uvs[2]);
    const cv::Vec2f uv02 = uv0 - uv2;

    cv::Vec2f ntuv;
    cv::normalize( cv::Vec2f( -uv02[1], uv02[0]), ntuv);  // Orthogonal normalised
    return uv2 + uv02*a + ntuv*o*cv::norm(uv02);
}   // end calcTextureCoord


// public
bool ObjModel::adjustVertex( int vidx, double x, double y, double z)
{
    if ( cvIsNaN( x) || cvIsNaN( y) || cvIsNaN( z))
        return false;

    if ( _vtxIds.count(vidx) == 0)
        return false;

    const cv::Vec3f nvec( (float)x, (float)y, (float)z);  // Updated position
    _verticesToUniqIdxs.erase( toKey( _verts[vidx], _fltPrc));  // Remove original vertex hash value
    _verts[vidx] = nvec;                                        // Update the position of the vertex
    _verticesToUniqIdxs[ toKey( _verts[vidx], _fltPrc)] = vidx; // Hash back with new vertices

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
    _edges[eidx] = Edge( v0, v1);
    _edgeMap[_edges[eidx]] = eidx;  // Reverse lookup
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
bool ObjModel::setOrderedFaceUVs( int materialID, int fid, const int vs[3], const cv::Vec2f uvs[3])
{
    if ( _materials.count(materialID) == 0)
        return false;

    Material& mat = *_materials[materialID];
#ifndef NDEBUG
    assert( getFaceIds().count(fid) > 0);
    assert( getVertexIds().count(vs[0]) && getVertexIds().count(vs[1]) && getVertexIds().count(vs[2]));
    assert( !cvIsNaN( uvs[0][0]) && !cvIsNaN( uvs[0][1]));
    assert( !cvIsNaN( uvs[1][0]) && !cvIsNaN( uvs[1][1]));
    assert( !cvIsNaN( uvs[2][0]) && !cvIsNaN( uvs[2][1]));
    // Can't have added previously to material
    assert( !mat.faceVertexOrder.count(fid));
    assert( !mat.faceUVOrder.count(fid));
#endif
    _faceMaterial[fid] = materialID;    // But CAN overwrite face's material ID! Necessary for mergeMaterials().
    mat.faceIds.insert(fid);
    mat.faceVertexOrder[fid] = cv::Vec3i( vs[0], vs[1], vs[2]);
    cv::Vec3i& fuvis = mat.faceUVOrder[fid];
    for ( int i = 0; i < 3; ++i)
    {
        Key2L key = toKey(uvs[i], _fltPrc);
        if ( mat._uvToUniqIdxs.count(key) == 0)
        {
            mat.uvIds.insert(mat._uvCounter);
            mat.uvs[mat._uvCounter] = uvs[i];
            mat._uvToUniqIdxs[key] = mat._uvCounter++;
        }   // end if
        fuvis[i] = mat.lookupUVIndex( uvs[i], _fltPrc);
        mat.uvFaceRefs[fuvis[i]].insert(fid);
    }   // end for

    return true;
}   // end setOrderedFaceUVs


// public
bool ObjModel::setOrderedFaceUVs( int materialID, int fid, int v0, const cv::Vec2f& uv0,
                                                           int v1, const cv::Vec2f& uv1,
                                                           int v2, const cv::Vec2f& uv2)
{
    const int vs[3] = {v0,v1,v2};
    const cv::Vec2f uvs[3] = {uv0,uv1,uv2};
    return setOrderedFaceUVs( materialID, fid, vs, uvs);
}   // end setOrderedFaceUVs


// public
const IntSet& ObjModel::getUVs( int materialID) const
{
    assert( _materials.count(materialID) > 0);
    return _materials.at(materialID)->uvIds;
}   // end getUVs


// public
const cv::Vec2f& ObjModel::uv( int materialID, int uvID) const
{
    assert( _materials.count(materialID) > 0);
    const Material& mat = *_materials.at(materialID);
    assert( mat.uvs.count(uvID) > 0);
    return mat.uvs.at(uvID);
}   // end uv


// public
const int* ObjModel::getFaceUVs( int fid) const
{
    assert( _faceIds.count(fid) > 0);
    const int mid = getFaceMaterialId(fid);
    if ( mid < 0)
        return NULL;
    const Material& mat = *_materials.at(mid);
    return &mat.faceUVOrder.at(fid)[0];
}   // end getFaceUVs


// public
const int* ObjModel::getFaceVertices( int fid) const
{
    if ( _faceIds.count(fid) == 0)
        return NULL;
    const int mid = getFaceMaterialId(fid);
    return mid >= 0 ? &_materials.at(mid)->faceVertexOrder.at(fid)[0] : getFace(fid).fvindices;
}   // end getFaceVertices


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
        removeFace( fid);    // May remove the edge if edge is shared by just one face

    if ( _edgeIds.count(edgeIdx))
        removeEdge( edgeIdx);

    return true;
}   // end unsetEdge


// public
int ObjModel::subDivideFace( int fidx, const cv::Vec3f& v)
{
    if ( _faces.count(fidx) == 0)
        return -1;

    const int nvidx = addVertex(v);   // New vertex added
    const int* vidxs = getFaceVertices(fidx);
    const int fid01 = setFace( nvidx, vidxs[0], vidxs[1]);
    const int fid12 = setFace( nvidx, vidxs[1], vidxs[2]);
    const int fid20 = setFace( nvidx, vidxs[2], vidxs[0]);

    // Set material if present
    const int matId = getFaceMaterialId( fidx);
    if ( matId >= 0)
    {
        const int* uvs = getFaceUVs( fidx);
        const cv::Vec2f& uv0 = uv( matId, uvs[0]);
        const cv::Vec2f& uv1 = uv( matId, uvs[1]);
        const cv::Vec2f& uv2 = uv( matId, uvs[2]);
        const cv::Vec2f uvn = calcTextureCoord( fidx, v);

        setOrderedFaceUVs( matId, fid01, vidxs[0], uv0, vidxs[1], uv1, nvidx, uvn);
        setOrderedFaceUVs( matId, fid12, vidxs[1], uv1, vidxs[2], uv2, nvidx, uvn);
        setOrderedFaceUVs( matId, fid20, vidxs[2], uv2, vidxs[0], uv0, nvidx, uvn);
    }   // end if

    // Finally, remove the old polygon.
    const bool removedOkay = removeFace(fidx);
    assert(removedOkay);
    return nvidx;
}   // end subDivideFace


// public
bool ObjModel::removeFace( int faceIdx)
{
    assert( _faces.count(faceIdx) > 0);
    if ( _faces.count(faceIdx) == 0)
        return false;

    const int* vids = getFaceVertices(faceIdx);
    unsetVertexFaceConnections( faceIdx, vids[0], vids[1], vids[2]);
    removeFaceEdges( faceIdx);

    // Remove from the Material if present
    const int matid = getFaceMaterialId( faceIdx);
    if ( matid >= 0)
        removeFaceUVs( matid, faceIdx);

    _faceMap.erase(_faces.at(faceIdx));
    _faces.erase(faceIdx);
    _faceIds.erase(faceIdx);
    return true;
}   // end removeFace


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

    const int* vids = getFaceVertices( faceId);
    int csum = -3;
    csum += (int)getFaceIds( vids[0]).size();
    csum += (int)getFaceIds( vids[1]).size();
    csum += (int)getFaceIds( vids[2]).size();
    // Remove 1 for each edge shared with a second face
    if ( getNumSharedFaces( vids[0], vids[1]) > 1) csum--;
    if ( getNumSharedFaces( vids[1], vids[2]) > 1) csum--;
    if ( getNumSharedFaces( vids[2], vids[0]) > 1) csum--;
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
    const int* vidxs = getFaceVertices(fid);

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
    static const IntSet EMPTY_INT_SET;
    if ( !_vtxConnectionFaces.count(vi))
        return EMPTY_INT_SET;
    const boost::unordered_map<int, IntSet>& uicf = _vtxConnectionFaces.at(vi);
    if ( !uicf.count(vj))
        return EMPTY_INT_SET;
    return uicf.at(vj);
}   // end getSharedFaces


// public
bool ObjModel::flipFacePair( int vi, int vj)
{
    const IntSet& sfids = getSharedFaces( vi, vj);
    if ( sfids.size() <= 1 || sfids.size() > 2)
        return false;

    const int f0 = *sfids.begin();
    const int f1 = *(++sfids.begin());
    const int vk = poly( f0).getOpposite( vi, vj);
    const int vl = poly( f1).getOpposite( vi, vj);
    // f0: i,j,k (i-->l)
    // f1: i,l,j (j-->k)
    // Update vertex to face mappings
    _vtxToFaces.at(vi).erase(f0);
    _vtxToFaces.at(vl).insert(f0);
    _vtxToFaces.at(vj).erase(f1);
    _vtxToFaces.at(vk).insert(f1);

    // vi and vj no longer connected
    _vtxConnectionFaces[vi].erase(vj);
    _vtxConnectionFaces[vj].erase(vi);

    // New edge connecting f0 and f1
    _vtxConnectionFaces[vk][vl].insert(f0);
    _vtxConnectionFaces[vl][vk].insert(f0);
    _vtxConnectionFaces[vk][vl].insert(f1);
    _vtxConnectionFaces[vl][vk].insert(f1);

    _vtxConnectionFaces[vi][vk].erase(f0);
    _vtxConnectionFaces[vk][vi].erase(f0);
    _vtxConnectionFaces[vi][vk].insert(f1);
    _vtxConnectionFaces[vk][vi].insert(f1);

    _vtxConnectionFaces[vl][vj].erase(f1);
    _vtxConnectionFaces[vj][vl].erase(f1);
    _vtxConnectionFaces[vl][vj].insert(f0);
    _vtxConnectionFaces[vj][vl].insert(f0);

    const int eij = getEdgeId(vi,vj);
    // Update face to edge mappings
    _faceEdgeIdxs[f0].erase(eij);
    _faceEdgeIdxs[f1].erase(eij);
    removeEdge(eij);    // Removes from _edgesToFaces

    int ekl = connectEdge( vk, vl);
    _faceEdgeIdxs[f0].insert(ekl);
    _faceEdgeIdxs[f1].insert(ekl);
    _edgesToFaces[ekl].insert(f0);
    _edgesToFaces[ekl].insert(f1);

    const int eik = getEdgeId(vi,vk);
    _faceEdgeIdxs[f0].erase(eik);
    _faceEdgeIdxs[f1].insert(eik);
    _edgesToFaces[eik].erase(f0);
    _edgesToFaces[eik].insert(f1);

    const int elj = getEdgeId(vl,vj);
    _faceEdgeIdxs[f0].insert(elj);
    _faceEdgeIdxs[f1].erase(elj);
    _edgesToFaces[elj].insert(f0);
    _edgesToFaces[elj].erase(f1);

    // Finally, need to reset the vertices in the ObjPolys!
    _faces.at(f0) = ObjPoly( vl, vj, vk);
    _faces.at(f1) = ObjPoly( vi, vl, vk);

    // Remap texture coords if necessary
    const int m0 = getFaceMaterialId(f0);
    const int m1 = getFaceMaterialId(f1);
    if ( m0 != m1)
    {
        // Remove texture coords if not the same!
        if (m0 >= 0)
            removeFaceUVs(m0, f0);
        if (m1 >= 0)
            removeFaceUVs(m1, f1);
    }   // end if
    else if ( m0 >= 0)
    {   
        // Faces share the same material ID (m0 or m1)
        Material& mat = *_materials.at(m0);

        cv::Vec3i& f0vtorder = mat.faceVertexOrder.at(f0); // Vertex order
        cv::Vec3i& f0uvorder = mat.faceUVOrder.at(f0);     // UV order
        cv::Vec3i& f1vtorder = mat.faceVertexOrder.at(f1); // Vertex order
        cv::Vec3i& f1uvorder = mat.faceUVOrder.at(f1);     // UV order
        int f0i, f1j;
        int uvi, uvj, uvk, uvl;
        for ( int i = 0; i < 3; ++i)
        {
            if ( f0vtorder[i] == vi)
            {
                f0vtorder[i] = vl;
                uvi = f0uvorder[i];
                f0i = i;
            }   // end if
            else if ( f0vtorder[i] == vk)
                uvk = f0uvorder[i];

            if ( f1vtorder[i] == vj)
            {
                f1vtorder[i] = vk;
                uvj = f1uvorder[i];
                f1j = i;
            }   // end if
            else if ( f1vtorder[i] == vl)
                uvl = f1uvorder[i];
        }   // end for

        mat.uvFaceRefs[uvi].erase(f0);
        mat.uvFaceRefs[uvj].erase(f1);
        mat.uvFaceRefs[uvk].insert(f1);
        mat.uvFaceRefs[uvl].insert(f0);

        f0uvorder[f0i] = uvl;
        f1uvorder[f1j] = uvk;
    }   // end else

    return true;
}   // end flipFacePair


// public
cv::Vec3f ObjModel::projectToPoly( int fid, const cv::Vec3f& v) const
{
    using namespace RFeatures;
    const int* vidxs = getFaceVertices(fid);
    const cv::Vec3f& v0 = vtx( vidxs[0]);
    const cv::Vec3f& v1 = vtx( vidxs[1]);
    const cv::Vec3f& v2 = vtx( vidxs[2]);

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
    const int* vidxs = getFaceVertices( fid);
    const cv::Vec3f& va = getVertex( vidxs[0]);
    const cv::Vec3f& vb = getVertex( vidxs[1]);
    const cv::Vec3f& vc = getVertex( vidxs[2]);

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
    static const IntSet EMPTY_INT_SET;
    if ( !_vtxToFaces.count(vi))
        return EMPTY_INT_SET;
    return _vtxToFaces.at(vi);
}   // end getFaceIds


// public
int ObjModel::lookupVertexIndex( const cv::Vec3f& v) const
{
    Key3L key = toKey( v, _fltPrc);
    return _verticesToUniqIdxs.count( key) > 0 ? _verticesToUniqIdxs.at(key) : -1;
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

    const IntSet& matIds = getMaterialIds();
    std::cerr << " Model has " << matIds.size() << " materials" << std::endl;
    BOOST_FOREACH ( int matId, matIds)
    {
        const Material& mat = *_materials.at(matId);
        std::cerr << " Material " << matId << " has [" << mat.ambient.size() << "," << mat.diffuse.size() << "," << mat.specular.size()
                  << "] ambient,diffuse,specular maps" << std::endl;
        std::cerr << " Material has " << mat.uvs.size() << " UV coordinates referencing it" << std::endl;
    }   // end foreach

    if ( withDetails)
    {
        std::cerr << " Face vertex & UV indices:" << std::endl;
        BOOST_FOREACH ( int fid, fids)
        {
            const int* vids = getFaceVertices(fid);
            std::cerr << "    F_" << fid << ") VTs: " << vids[0] << ", " << vids[1] << ", " << vids[2] << std::endl;
            const int mid = getFaceMaterialId(fid);
            if ( mid >= 0)
            {
                const int* uvis = getFaceUVs(fid);
                std::cerr << std::right << std::setw(int(log10(std::max(1,fid))) + 14)
                                        << " UVs: " << uvis[0] << "=" << uv(mid, uvis[0])
                                            << ", " << uvis[1] << "=" << uv(mid, uvis[1])
                                            << ", " << uvis[2] << "=" << uv(mid, uvis[2])
                                            << "  on Mat " << mid << std::endl;
            }   // end if
        }   // end for
        std::cerr << std::endl;
        std::cerr << " --------------------------------------------------- " << std::endl;
    }   // end if

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
