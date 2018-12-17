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
#include <boost/functional/hash.hpp>
#include <FeatureUtils.h>

using RFeatures::ObjPoly;
using RFeatures::Edge;
using RFeatures::ObjModel;
using std::unordered_map;


namespace {
static const IntSet EMPTY_INT_SET;

void reorderMinToMax( int &v0, int &v1, int &v2)
{
    if ( v0 > v1)
        std::swap( v0, v1);
    if ( v1 > v2)
        std::swap( v1, v2);
    if ( v0 > v1)
        std::swap( v0, v1);
}   // end reorderMinToMax
}   // end namespace


ObjPoly::ObjPoly() {}

ObjPoly::ObjPoly( int v0, int v1, int v2)
{
    fvindices[0] = v0;
    fvindices[1] = v1;
    fvindices[2] = v2;
}   // end ctor


// Two ObjPolys are the same if they have the same vertices
bool ObjPoly::operator==( const ObjPoly& p) const
{
    int v0 = fvindices[0];
    int v1 = fvindices[1];
    int v2 = fvindices[2];
    reorderMinToMax( v0, v1, v2);

    int p0 = p.fvindices[0];
    int p1 = p.fvindices[1];
    int p2 = p.fvindices[2];
    reorderMinToMax( p0, p1, p2);

    return (v0 == p0) && (v1 == p1) && (v2 == p2);
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
        other0 = fvindices[2];
        other1 = fvindices[0];
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


// public
int ObjPoly::getIndex( int vidx) const
{
    int i = -1;
    if ( fvindices[0] == vidx)
        i = 0;
    else if ( fvindices[1] == vidx)
        i = 1;
    else if ( fvindices[2] == vidx)
        i = 2;
    return i;
}   // end getIndex


Edge::Edge() {}

Edge::Edge( int u0, int u1) : v0(u0), v1(u1)
{
    assert( v0 != v1);
    if ( v1 < v0)   // Ensure v0 is always smaller
        std::swap( v0, v1);
}   // end ctor

bool Edge::operator==( const Edge& e) const
{
    return e.v0 == v0 && e.v1 == v1;    // Requires vertices to be stored in ascending order
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
    unordered_map<int, cv::Vec2f> uvs;          // UV IDs to UVs 
    unordered_map<int, IntSet> uvFaceRefs;      // UV IDs --> Face IDs 

    IntSet faceIds;                             // All face IDs mapped to this material
    unordered_map<int, cv::Vec3i> faceUVOrder;  // Face IDs --> UV IDs (matching order of vertices in ObjPoly)

    Key2LToIntMap _uvToUniqIdxs;                // How UVs map to the keys of uvs
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

    void mapUV( int fid, const cv::Vec2f& uv, int fltPrc)
    {
        Key2L key = toKey(uv, fltPrc);
        if ( _uvToUniqIdxs.count(key) == 0)
        {
            uvIds.insert(_uvCounter);
            uvs[_uvCounter] = uv;
            _uvToUniqIdxs[key] = _uvCounter++;
        }   // end if
    }   // end mapUV

private:
    Material( const Material&) = delete;
    void operator=( const Material&) = delete;
};  // end class


// public
size_t ObjModel::numTextureEdges( int va, int vb) const
{
    const IntSet& sfids = getSharedFaces( va, vb);
    if ( sfids.empty())
        return 0;

    size_t ntex = 0;
    IntSet tset;    // Records the texture vertices
    for ( int fid : sfids)
    {
        const int* uvs = faceUVs( fid);
        if ( !uvs)
            continue;

        int a = 0;  // First edge vertex index
        int b = 1;  // Second edge vertex index
        const int* vidxs = fvidxs( fid);    // Corresponding vertices
        if ( vidxs[0] == va)
        {
            a = 0;
            b = vidxs[1] == vb ? 1 : 2;
        }   // end if
        else if ( vidxs[1] == va)
        {
            a = 1;
            b = vidxs[0] == vb ? 0 : 2;
        }   // end else if
        else
        {
            a = 2;
            b = vidxs[0] == vb ? 0 : 1;
        }   // end else if

        assert( vidxs[a] == va);
        assert( vidxs[b] == vb);

        // vidxs[a] == va and vidxs[b] == vb, so uvs[a] and uvs[b] is the corresponding texture edge.
        // If either of the texture vertices are new, it counts as a new texture edge.
        if ( tset.count(uvs[a]) == 0 || tset.count(uvs[b]) == 0)
            ntex++;
        tset.insert(uvs[a]);
        tset.insert(uvs[b]);
    }   // end foreach

    return ntex;
}   // end numTextureEdges


// public static
ObjModel::Ptr ObjModel::copy( const ObjModel* omc, bool shareMats)
{
    ObjModel::Ptr nm = create( omc->_fltPrc);

    // Copy over the vertices from the old model
    for ( int vidx : omc->getVertexIds())
    {
        const cv::Vec3f& v = omc->vtx(vidx);
        nm->addVertex( vidx, v[0], v[1], v[2]);
    }   // end for
    nm->_vCounter = omc->_vCounter;

    // Copy over materials
    const IntSet& matIds = omc->getMaterialIds();
    for ( int mid : matIds)
    {
        nm->addMaterial(mid);

        for ( const cv::Mat m : omc->materialAmbient(mid))
            nm->addMaterialAmbient(  mid, shareMats ? m : m.clone());
        for ( const cv::Mat m : omc->materialDiffuse(mid))
            nm->addMaterialDiffuse(  mid, shareMats ? m : m.clone());
        for ( const cv::Mat m : omc->materialSpecular(mid))
            nm->addMaterialSpecular( mid, shareMats ? m : m.clone());
    }   // end for
    nm->_mCounter = omc->_mCounter;

    // Copy over the faces
    const IntSet& fids = omc->getFaceIds();
    for ( int fid : fids)
    {
        const int* vids = omc->poly(fid).fvindices;
        const int e0 = omc->edgeId( vids[0], vids[1]);
        const int e1 = omc->edgeId( vids[1], vids[2]);
        const int e2 = omc->edgeId( vids[2], vids[0]);

        nm->connectEdge( e0, vids[0], vids[1]);
        nm->connectEdge( e1, vids[1], vids[2]);
        nm->connectEdge( e2, vids[2], vids[0]);
        nm->addFace( fid, vids[0], vids[1], vids[2], e0, e1, e2);

        const int mid = omc->faceMaterialId( fid);
        if ( mid >= 0)
        {
            const int* uvids = omc->faceUVs(fid);
            nm->setOrderedFaceUVs( mid, fid, omc->uv(mid, uvids[0]), omc->uv(mid, uvids[1]), omc->uv(mid, uvids[2]));
        }   // end if
    }   // end for
    nm->_eCounter = omc->_eCounter;
    nm->_fCounter = omc->_fCounter;

    return nm;
}   // end copy


// public static
ObjModel::Ptr ObjModel::create( int fltPrc)
{
    return Ptr( new ObjModel(fltPrc), [](ObjModel* x){delete x;});
}   // end create


// private
ObjModel::ObjModel( int fltPrc)
    :  _fltPrc( fltPrc), _vCounter(0), _fCounter(0), _eCounter(0), _mCounter(0)
{}   // end ctor


// private
ObjModel::~ObjModel()
{
    removeAllMaterials();
}   // end dtor


// public
int ObjModel::faceMaterialId( int fid) const
{
    assert( _faceIds.count(fid) > 0);
    if ( _faceMaterial.count(fid) == 0) // No material for this face
        return -1;
    return _faceMaterial.at(fid);
}   // end faceMaterialId


// public
const IntSet& ObjModel::materialFaceIds( int mid) const
{
    if ( _materials.count(mid) == 0)
        return EMPTY_INT_SET;
    return _materials.at(mid)->faceIds;
}   // end materialFaceIds


// public
int ObjModel::addMaterial()
{
    const int mid = _mCounter++;
    addMaterial( mid);
    return mid;
}   // end addMaterial


// private
void ObjModel::addMaterial( int mid)
{
    _materialIds.insert(mid);
    _materials[mid] = new Material();
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
    std::for_each( std::begin(mids), std::end(mids), [this](int mid){ removeMaterial(mid);});
    return true;
}   // end removeAllMaterials


namespace {
// Calculate a new UV coordinate from an old UV.
void calcNewUV( cv::Vec2f& uv, int nrows, int ncols, const std::vector<int>& scols, int i)
{
    // v is unchanged, only u affected
    const double oldWidth = (i == (int)(scols.size())-1) ? ncols - scols[i] : scols[i+1] - scols[i];
    uv[0] = (float)(scols[i] + (uv[0] * oldWidth))/ncols;
}   // end calcNewUV
}   // end namespace


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
    for ( int mid : mids)
    {
        midSeq.push_back(mid);
        const std::vector<cv::Mat>& ams = materialAmbient(mid);
        const std::vector<cv::Mat>& dms = materialDiffuse(mid);
        const std::vector<cv::Mat>& sms = materialSpecular(mid);
        if ( !ams.empty())
            aimgs.push_back(ams[0]);
        if ( !dms.empty())
            dimgs.push_back(dms[0]);
        if ( !sms.empty())
            simgs.push_back(sms[0]);
    }   // end for

    std::vector<int> scols; // The starting columns for the concatenated texture images
    const cv::Mat aimg = concatHorizontalMax( aimgs, &scols);
    const cv::Mat dimg = concatHorizontalMax( dimgs, aimg.empty() ? &scols : nullptr);
    const cv::Mat simg = concatHorizontalMax( simgs, dimg.empty() ? &scols : nullptr);
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
        const IntSet& fids = materialFaceIds( mid);
        for ( int fid : fids)
        {
            // Get and set the new texture offsets for the face based on
            // the horizontal concatentation of the texture images.
            const int* uvidxs = faceUVs(fid);

            cv::Vec2f uv0 = uv(mid, uvidxs[0]);
            cv::Vec2f uv1 = uv(mid, uvidxs[1]);
            cv::Vec2f uv2 = uv(mid, uvidxs[2]);

            // Calculate new offsets
            calcNewUV( uv0, nrows, ncols, scols, i);
            calcNewUV( uv1, nrows, ncols, scols, i);
            calcNewUV( uv2, nrows, ncols, scols, i);

            // Set in the merged material
            setOrderedFaceUVs( mmid, fid, uv0, uv1, uv2);
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
    _materials[materialID]->ambient.push_back( shrinkMax( m, maxd));
    return true;
}   // end addMaterialAmbient


// public
bool ObjModel::addMaterialDiffuse( int materialID, const cv::Mat& m, size_t maxd)
{
    if ( _materials.count( materialID) == 0 || m.empty())
        return false;
    _materials[materialID]->diffuse.push_back( shrinkMax( m, maxd));
    return true;
}   // end addMaterialDiffuse


// public
bool ObjModel::addMaterialSpecular( int materialID, const cv::Mat& m, size_t maxd)
{
    if ( _materials.count( materialID) == 0 || m.empty())
        return false;
    _materials[materialID]->specular.push_back( shrinkMax( m, maxd));
    return true;
}   // end addMaterialSpecular


// public
const std::vector<cv::Mat>& ObjModel::materialAmbient( int mid) const
{
    assert( _materials.count(mid) > 0);
    return _materials.at(mid)->ambient;
}   // end materialAmbient


// public
const std::vector<cv::Mat>& ObjModel::materialDiffuse( int mid) const
{
    assert( _materials.count(mid) > 0);
    return _materials.at(mid)->diffuse;
}   // end materialDiffuse


// public
const std::vector<cv::Mat>& ObjModel::materialSpecular( int mid) const
{
    assert( _materials.count(mid) > 0);
    return _materials.at(mid)->specular;
}   // end materialSpecular


// private
void ObjModel::removeFaceUVs( int matid, int fid)
{
    assert( getMaterialIds().count( matid) > 0);
    Material& m = *_materials.at( matid);

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

    m.faceUVOrder.erase(fid);
    m.faceIds.erase(fid);
    _faceMaterial.erase(fid);
}   // end removeFaceUVs


// public
int ObjModel::addVertex( float x, float y, float z)
{
    if ( cvIsNaN( x) || cvIsNaN( y) || cvIsNaN( z))
        return -1;

    Key3L key = toKey( x, y, z, _fltPrc);
    if ( _verticesToUniqIdxs.count( key) > 0)
        return _verticesToUniqIdxs.at(key);

    const int vi = _vCounter++;
    addVertex( vi, x, y, z);
    return vi;
}   // end addVertex


// private
void ObjModel::addVertex( int vi, float x, float y, float z)
{
    assert( _vtxIds.count(vi) == 0);

    Key3L key = toKey( x, y, z, _fltPrc);
    assert( _verticesToUniqIdxs.count( key) == 0);
    _verticesToUniqIdxs[key] = vi;

    _vtxIds.insert(vi);
    _verts[vi] = cv::Vec3f( x, y, z);
    _vtxConnections[vi].clear();
    _vtxConnectionFaces[vi].clear();
    _vtxToFaces[vi].clear();
    _vtxToEdges[vi].clear();
}   // end addVertex


// public
int ObjModel::addVertex( const cv::Vec3f& v)
{
    return addVertex( v[0], v[1], v[2]);
}   // end addVertex


// public
bool ObjModel::removeVertex( int vi)
{
    assert( getVertexIds().count(vi) > 0);

    // Check that this vertex is no longer connected to others
    // i.e., connecting faces have been removed.
    const IntSet& cvtxs = getConnectedVertices(vi);
    assert(cvtxs.empty());
    if ( !cvtxs.empty())
        return false;

    assert( getFaceIds(vi).empty());

    const cv::Vec3f& v = _verts.at(vi);
    Key3L key = toKey( v[0], v[1], v[2], _fltPrc);
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
cv::Vec2f ObjModel::calcTextureCoords( int fidx, const cv::Vec3f& p) const
{
    const int matId = faceMaterialId( fidx);
    if ( matId < 0)
        return cv::Vec2f(-1,-1);

    const int* vidxs = fvidxs(fidx);
    const cv::Vec3f& v0 = vtx(vidxs[0]);
    const cv::Vec3f& v1 = vtx(vidxs[1]);
    const cv::Vec3f& v2 = vtx(vidxs[2]);

    const double C = cv::norm(v2-v1);   // Picture C as base of triangle with v2 bottom left, v1 bottom right (v0 at top).
    assert(C > 0.0);
    const double Y = cv::norm(p-v1);
    const double Z = cv::norm(p-v2);
    const double b = 2.0*calcTriangleArea(Y,Z,C)/C;   // b is height of triangle YZC and is at right angles to line segment C

    // Calculate G: the scaling constant for the triangle areas.
    const int* uvs = faceUVs( fidx);
    const cv::Vec2f& t0 = uv( matId, uvs[0]);
    const cv::Vec2f& t1 = uv( matId, uvs[1]);
    const cv::Vec2f& t2 = uv( matId, uvs[2]);
    const double AT = cv::norm(t2-t0);
    const double BT = cv::norm(t1-t0);
    const double CT = cv::norm(t2-t1);
    double G = 1.0;
    const double garea = calcTriangleArea(v0,v1,v2);
    const double tarea = calcTriangleArea(AT,BT,CT);
    if ( garea > 0.0)
        G = tarea / garea;
    else if ( tarea > 0.0)
        G = 0.0;

    // For the geometry triangle and the texture mapped triangle, the ratios of sub-areas to the whole triangle area must match.
    // That is, for sub-region R on the geometry triangle with total area G, there must be a sub-region S on the texture triangle
    // with total area T such that R/G = S/T. Given that S is of the same proportion, it must hold the requisite texture portion to map.
    double bT = 0.0;
    if (CT > 0.0)
        bT = G*b*C/CT;

    // b*sqrt(Y^2 - b^2)/(2*Area(ABC)) = bT*sqrt(YT^2-bT^2)/(2*AreaT(ABC))
    // After algebra, YT^2 = G^2 * b^2/bT^2 * (Y^2 - b^2) + bT^2
    // Therefore, by Pythagoras, the desired length along CT from t1 is sqrt(G^2 * b^2/bT^2 * (Y^2 - b^2)).
    // However, if bT is zero, b^2/bT^2 == 1.
    const double bSq = b*b;
    double bratio = 1.0;
    if ( bT > 0.0)
        bratio = bSq / (bT*bT);
    const double CTsubLen = sqrt( std::max( 0.0, G*G * bratio * (Y*Y - bSq)));

    cv::Vec2f unitVecC; // Direction vector along texture triangle line segment.
    cv::normalize( t2-t1, unitVecC);    // Use the OpenCV function to account for possibility of CT == 0.0
    // unitVecC needs scaling by CTsubLen and an orthogonal vector (toward t0) scaled to length bT added to
    // give the texture coordinates with respect to texture vertex t1.
    // Depending on texture mapping, either unitVecB or unitVecB2 will be the correct orthogonal vector to unitVecC.
    // Test by seeing which (when added to t1) is closer to t0.
    cv::Vec2f unitVecB( -unitVecC[1], unitVecC[0]);
    const cv::Vec2f unitVecB2( unitVecC[1], -unitVecC[0]);
    const double delta0 = l2sq( t1 + unitVecB - t0);
    const double delta1 = l2sq( t1 + unitVecB2 - t0);
    if ( delta1 < delta0)
        unitVecB = unitVecB2;

    const cv::Vec2f ov = t1 + (float(CTsubLen) * unitVecC) + (float(bT) * unitVecB);
    assert( !cvIsNaN( ov[0]) && !cvIsNaN( ov[1]));
    return ov;
}   // end calcTextureCoords


// public
bool ObjModel::adjustVertex( int vidx, float x, float y, float z)
{
    if ( cvIsNaN( x) || cvIsNaN( y) || cvIsNaN( z))
        return false;

    if ( _vtxIds.count(vidx) == 0)
        return false;

    cv::Vec3f& vec = _verts.at(vidx);   // The vertex to modify

    // Remove original vertex hash value
    _verticesToUniqIdxs.erase( toKey( vec[0], vec[1], vec[2], _fltPrc));

    // Update with new position of vertex
    vec[0] = x;
    vec[1] = y;
    vec[2] = z;
    _verticesToUniqIdxs[ toKey( x, y, z, _fltPrc)] = vidx;  // Hash back with new vertices

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
    assert( (gotEdge && edgeId(v0,v1) >= 0) || (!gotEdge && edgeId(v0,v1) < 0));
    return gotEdge;
}   // end hasEdge


// public
int ObjModel::edgeId( int v0, int v1) const
{
    const Edge lookupEdge( v0, v1);
    if ( _edgeMap.count(lookupEdge) > 0)
        return _edgeMap.at(lookupEdge);
    return -1;
}   // end edgeId


// private
int ObjModel::connectEdge( int v0, int v1)
{
    assert( v0 != v1);
    int eidx = edgeId( v0, v1);
    if ( eidx >= 0)
        return eidx;
    eidx = _eCounter++;
    connectEdge( eidx, v0, v1);
    return eidx;
}   // end connectEdge


// private
void ObjModel::connectEdge( int eidx, int v0, int v1)
{
    assert( _edges.count(eidx) == 0);
    _vtxConnections[v0].insert(v1);
    _vtxConnections[v1].insert(v0);
    _edges[eidx] = Edge( v0, v1);
    _edgeMap[_edges[eidx]] = eidx;  // Reverse lookup
    _edgeIds.insert(eidx);
    _vtxToEdges[v0].insert(eidx);
    _vtxToEdges[v1].insert(eidx);
}   // end connectEdge


// public
int ObjModel::setEdge( int v0, int v1)
{
    int eidx = edgeId( v0, v1);
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
    for ( int ui : *uset)
    {
        if ( sset->count(ui) > 0)
            ucs.push_back(ui);
    }   // end foreach

    eidx = connectEdge( v0, v1);
    // Create the polygon(s)
    for ( int v : ucs)
        addFace( v, v0, v1);
    return eidx;
}   // end setEdge


// public
int ObjModel::getFaceId( int v0, int v1, int v2) const
{
    const ObjPoly lookupPoly( v0, v1, v2);
    if ( _faceMap.count(lookupPoly) > 0)
        return _faceMap.at(lookupPoly);
    return -1;
}   // end getFaceId


// public
int ObjModel::addFace( const int* vtxs) { return addFace( vtxs[0], vtxs[1], vtxs[2]);}


// public
int ObjModel::addFace( int v0, int v1, int v2)
{
    const IntSet& vidxs = getVertexIds();
    if ( vidxs.count(v0) == 0 || vidxs.count(v1) == 0 || vidxs.count(v2) == 0)
        return -1;

    int fidx = getFaceId( v0, v1, v2); // Don't add if already present
    if ( fidx >= 0)
        return fidx;

    fidx = _fCounter++;
    const int e0 = connectEdge( v0, v1);
    const int e1 = connectEdge( v1, v2);
    const int e2 = connectEdge( v2, v0);
    addFace( fidx, v0, v1, v2, e0, e1, e2);

    return fidx;
}   // end addFace


// private
void ObjModel::addFace( int fidx, int v0, int v1, int v2, int e0, int e1, int e2)
{
    _faceEdgeIdxs[fidx].insert( e0);
    _edgesToFaces[e0].insert(fidx);

    _faceEdgeIdxs[fidx].insert( e1);
    _edgesToFaces[e1].insert(fidx);

    _faceEdgeIdxs[fidx].insert( e2);
    _edgesToFaces[e2].insert(fidx);

    _vtxToFaces[v0].insert(fidx);
    _vtxToFaces[v1].insert(fidx);
    _vtxToFaces[v2].insert(fidx);

    _vtxConnectionFaces[v0][v1].insert(fidx);
    _vtxConnectionFaces[v0][v2].insert(fidx);
    _vtxConnectionFaces[v1][v0].insert(fidx);
    _vtxConnectionFaces[v1][v2].insert(fidx);
    _vtxConnectionFaces[v2][v0].insert(fidx);
    _vtxConnectionFaces[v2][v1].insert(fidx);

    _faceIds.insert( fidx);
    _faceMap[_faces[fidx] = ObjPoly( v0, v1, v2)] = fidx;
}   // end addFace


// public
bool ObjModel::setOrderedFaceUVs( int materialID, int fid, const cv::Vec2f& uv0, const cv::Vec2f& uv1, const cv::Vec2f& uv2)
{
    if ( _materials.count(materialID) == 0)
        return false;

    Material& mat = *_materials[materialID];
#ifndef NDEBUG
    assert( getFaceIds().count(fid) > 0);
    assert( !cvIsNaN( uv0[0]) && !cvIsNaN( uv0[1]));
    assert( !cvIsNaN( uv1[0]) && !cvIsNaN( uv1[1]));
    assert( !cvIsNaN( uv2[0]) && !cvIsNaN( uv2[1]));
    // Can't have added previously to material
    assert( mat.faceUVOrder.count(fid) == 0);
#endif
    _faceMaterial[fid] = materialID;    // But CAN overwrite face's material ID! Necessary for mergeMaterials().
    mat.faceIds.insert(fid);

    mat.mapUV( fid, uv0, _fltPrc);
    mat.mapUV( fid, uv1, _fltPrc);
    mat.mapUV( fid, uv2, _fltPrc);

    cv::Vec3i& fuvis = mat.faceUVOrder[fid];
    fuvis[0] = mat.lookupUVIndex( uv0, _fltPrc);
    fuvis[1] = mat.lookupUVIndex( uv1, _fltPrc);
    fuvis[2] = mat.lookupUVIndex( uv2, _fltPrc);

    mat.uvFaceRefs[fuvis[0]].insert(fid);
    mat.uvFaceRefs[fuvis[1]].insert(fid);
    mat.uvFaceRefs[fuvis[2]].insert(fid);

    return true;
}   // end setOrderedFaceUVs


// public
bool ObjModel::setOrderedFaceUVs( int materialID, int fid, const cv::Vec2f uvs[3])
{
    return setOrderedFaceUVs( materialID, fid, uvs[0], uvs[1], uvs[2]);
}   // end setOrderedFaceUVs


// public
const IntSet& ObjModel::uvs( int materialID) const
{
    assert( _materials.count(materialID) > 0);
    return _materials.at(materialID)->uvIds;
}   // end uvs


// public
const cv::Vec2f& ObjModel::uv( int materialID, int uvID) const
{
    assert( _materials.count(materialID) > 0);
    const Material& mat = *_materials.at(materialID);
    assert( mat.uvs.count(uvID) > 0);
    return mat.uvs.at(uvID);
}   // end uv


// public
const int* ObjModel::faceUVs( int fid) const
{
    assert( _faceIds.count(fid) > 0);
    const int mid = faceMaterialId(fid);
    if ( mid < 0)
        return nullptr;
    const Material& mat = *_materials.at(mid);
    return &mat.faceUVOrder.at(fid)[0];
}   // end faceUVs


// public
const int* ObjModel::fvidxs( int fid) const
{
    assert( _faceIds.count(fid) > 0);
    if ( _faceIds.count(fid) == 0)
        return nullptr;
    return _faces.at(fid).fvindices;
}   // end fvidxs


// public
void ObjModel::reverseFaceVertices( int fid)
{
    assert(_faces.count(fid) > 0);
    ObjPoly& poly = _faces.at(fid);
    std::swap( poly.fvindices[0], poly.fvindices[2]);

    const int mid = faceMaterialId( fid);
    if ( mid >= 0)
    {
        Material& mat = *_materials.at(mid);
        cv::Vec3i& fuvis = mat.faceUVOrder.at(fid);
        std::swap( fuvis[0], fuvis[2]);
    }   // end if
}   // end reverseFaceVertices


// public
cv::Vec3f ObjModel::calcFaceNorm( int fid) const
{
    const int* vidxs = fvidxs(fid);
    assert(vidxs);
    const cv::Vec3f vi = vtx(vidxs[1]) - vtx(vidxs[0]);
    const cv::Vec3f vj = vtx(vidxs[2]) - vtx(vidxs[1]);
    cv::Vec3f fn;
    cv::normalize( vi.cross(vj), fn);
    return fn;
}   // end calcFaceNorm


// public
cv::Vec3f ObjModel::calcFaceNorm( int fid, cv::Vec3f& vi, cv::Vec3f& vj) const
{
    const int* vidxs = fvidxs(fid);
    assert(vidxs);
    cv::normalize( vtx(vidxs[1]) - vtx(vidxs[0]), vi);
    cv::normalize( vtx(vidxs[2]) - vtx(vidxs[1]), vj);
    cv::Vec3f fn;
    cv::normalize( vi.cross(vj), fn);
    return fn;
}   // end calcFaceNorm


// private
void ObjModel::removeEdge( int eidx)
{
    const Edge& e = getEdge(eidx);
    _vtxToEdges[e.v0].erase(eidx);
    _vtxToEdges[e.v1].erase(eidx);
    _vtxConnections[e.v0].erase(e.v1);
    _vtxConnections[e.v1].erase(e.v0);
    // No longer using e
    _edgeIds.erase(eidx);
    _edgeMap.erase(_edges.at(eidx));
    _edges.erase(eidx);
    _edgesToFaces.erase(eidx);
}   // end removeEdge


// public
bool ObjModel::unsetEdge( int edgeIdx)
{
    if ( _edgeIds.count(edgeIdx) == 0)
        return false;

    const Edge& e = getEdge( edgeIdx);
    // Removing an edge removes all attached faces
    const IntSet fids = getSharedFaces( e.v0, e.v1);    // Copy out
    std::for_each( std::begin(fids), std::end(fids), [this](int fid){ removeFace(fid);}); // May remove edge if shared by single face
    if ( _edgeIds.count(edgeIdx) > 0)
        removeEdge( edgeIdx);
    return true;
}   // end unsetEdge


// public
bool ObjModel::unsetEdge( int vi, int vj) { return unsetEdge( edgeId( vi, vj));}


// public
int ObjModel::subDivideFace( int fidx, const cv::Vec3f& v)
{
    if ( _faces.count(fidx) == 0)
        return -1;

    const int nvidx = addVertex(v);   // New vertex added
    const int* vidxs = fvidxs(fidx);
    // These addFace orderings will ensure that the subdivided faces
    // have the same direction normal as the parent face being subdivided.
    const int fid01 = addFace( nvidx, vidxs[0], vidxs[1]);
    const int fid12 = addFace( nvidx, vidxs[1], vidxs[2]);
    const int fid20 = addFace( nvidx, vidxs[2], vidxs[0]);

    // Set material if present
    const int matId = faceMaterialId( fidx);
    if ( matId >= 0)
    {
        const int* uvs = faceUVs( fidx);
        const cv::Vec2f& uv0 = uv( matId, uvs[0]);
        const cv::Vec2f& uv1 = uv( matId, uvs[1]);
        const cv::Vec2f& uv2 = uv( matId, uvs[2]);
        const cv::Vec2f uvn = calcTextureCoords( fidx, v);

        setOrderedFaceUVs( matId, fid01, uv0, uv1, uvn);
        setOrderedFaceUVs( matId, fid12, uv1, uv2, uvn);
        setOrderedFaceUVs( matId, fid20, uv2, uv0, uvn);
    }   // end if

    // Finally, remove the old polygon.
    removeFace(fidx);
    return nvidx;
}   // end subDivideFace


// public
int ObjModel::subDivideFace( int fidx, int *nfidxs)
{
    if ( _faces.count(fidx) == 0)
        return -1;

    const int* vidxs = fvidxs( fidx);

    // Get the three new vertex positions
    const cv::Vec3f v0 = (vtx(vidxs[0]) + vtx(vidxs[1])) * 0.5f;
    const cv::Vec3f v1 = (vtx(vidxs[1]) + vtx(vidxs[2])) * 0.5f;
    const cv::Vec3f v2 = (vtx(vidxs[2]) + vtx(vidxs[0])) * 0.5f;

    // Add the three new vertices...
    const int nv0 = addVertex( v0);
    const int nv1 = addVertex( v1);
    const int nv2 = addVertex( v2);

    const int nf0 = addFace( nv0, nv1, nv2);  // ... add the new centre face ...
    // ... and the new faces adjacent to it.
    const int nf1 = addFace( vidxs[1], nv1, nv0);
    const int nf2 = addFace( vidxs[2], nv2, nv1);
    const int nf3 = addFace( vidxs[0], nv0, nv2);
    // NB the above new faces have vertex orders that maintain the normal directions of the face being subdivided.

    const int matId = faceMaterialId( fidx);
    if ( matId >= 0)
    {
        const cv::Vec2f uv0 = calcTextureCoords( fidx, v0);
        const cv::Vec2f uv1 = calcTextureCoords( fidx, v1);
        const cv::Vec2f uv2 = calcTextureCoords( fidx, v2);
        setOrderedFaceUVs( matId, nf0, uv2, uv0, uv1);

        const int* uvs = faceUVs( fidx);
        setOrderedFaceUVs( matId, nf1, uv0, uv( matId, uvs[1]), uv1);
        setOrderedFaceUVs( matId, nf2, uv1, uv( matId, uvs[2]), uv2);
        setOrderedFaceUVs( matId, nf3, uv2, uv( matId, uvs[0]), uv0);
    }   // end if

    removeFace(fidx);

    if ( nfidxs) // Copy out the newly added face IDs into user provided storage (if given).
    {
        nfidxs[0] = nf0;
        nfidxs[1] = nf1;
        nfidxs[2] = nf2;
        nfidxs[3] = nf3;
    }   // end if

    return nf0;
}   // end subDivideFace


// public
bool ObjModel::subDivideEdge( int vi, int vj, int vn)
{
    const IntSet& sfids = getSharedFaces( vi, vj);
    if ( sfids.empty())
        return false;

    assert( vn >= 0 && vn != vi && vn != vj);
    for ( int fid : sfids)
    {
        const int* vidxs = fvidxs(fid);

        // Create two new faces
        // Order of vidxs will be clockwise and match the order of uvs.
        // This order needs to be maintained so that polygon normals don't flip.
        const int vk = poly(fid).getOpposite(vi,vj);    // Vertex on the shared face that isn't the edge vertex
        // Find the right ordering of vi and vj so that vi is immediately after vk and vj is after vi.
        int k = 0;
        int i = 1;
        int j = 2;
        if ( vidxs[0] == vk)
        {
            vi = vidxs[1];
            vj = vidxs[2];
        }   // end if
        else if ( vidxs[1] == vk)
        {
            k = 1;
            i = 2;
            j = 0;
            vi = vidxs[2];
            vj = vidxs[0];
        }   // end else if
        else
        {
            k = 2;
            i = 0;
            j = 1;
            vi = vidxs[0];
            vj = vidxs[1];
        }   // end else

        const int f0 = addFace( vn, vk, vi);
        const int f1 = addFace( vn, vj, vk);

        // Set material if present
        const int mid = faceMaterialId( fid);
        if ( mid >= 0)
        {
            const int* uvs = faceUVs( fid);
            const cv::Vec2f& uvk = uv(mid, uvs[k]);
            const cv::Vec2f& uvi = uv(mid, uvs[i]);
            const cv::Vec2f& uvj = uv(mid, uvs[j]);
            const cv::Vec2f uvn = calcTextureCoords( fid, vtx(vn));
            setOrderedFaceUVs( mid, f0, uvn, uvk, uvi);
            setOrderedFaceUVs( mid, f1, uvn, uvj, uvk);
        }   // end if
    }   // end foreach

    const int eid = edgeId( vi, vj);
    unsetEdge( eid); // Finally, remove the old polygons attached to the edge
    return true;
}   // end subDivideEdge


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

    // Remove the entries if the connection between vertices shares no more faces.
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
void ObjModel::removeFaceEdges( int faceIdx)
{
    assert( _faceEdgeIdxs.count(faceIdx) > 0);
    const IntSet edgeIds = _faceEdgeIdxs.at(faceIdx);   // Copy out
    for ( int eidx : edgeIds)
    {
#ifndef NDEBUG
        int v0, v1;
        assert( getEdge( eidx, v0, v1));
#endif
        _faceEdgeIdxs[faceIdx].erase(eidx);
        _edgesToFaces[eidx].erase(faceIdx);
        if ( _edgesToFaces.at(eidx).empty())    // An edge is only removed if no other faces are shared with it
        {
#ifndef NDEBUG
            assert( getSharedFaces(v0,v1).count(faceIdx) == 0);
#endif
            removeEdge( eidx);
        }   // end if
    }   // end foreach
    _faceEdgeIdxs.erase(faceIdx);
}   // end removeFaceEdges


// public
bool ObjModel::removeFace( int faceIdx)
{
    assert( _faces.count(faceIdx) > 0);
    if ( _faces.count(faceIdx) == 0)
        return false;

    const int* vids = fvidxs(faceIdx);
    assert( vids);

    unsetVertexFaceConnections( faceIdx, vids[0], vids[1], vids[2]);

    removeFaceEdges( faceIdx);

    // Remove from the Material if present
    const int matid = faceMaterialId( faceIdx);
    if ( matid >= 0)
        removeFaceUVs( matid, faceIdx);

    _faceMap.erase(_faces.at(faceIdx));
    _faces.erase(faceIdx);
    _faceIds.erase(faceIdx);
    return true;
}   // end removeFace


// public
int ObjModel::getFaceConnectivityMetric( int faceId) const
{
    if ( _faceIds.count(faceId) == 0)
        return -1;

    const int* vids = fvidxs( faceId);
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
bool ObjModel::getEdge( int edgeId, int& v0, int& v1) const
{
    if ( _edges.count(edgeId) == 0)
        return false;
    const Edge& e = getEdge(edgeId);
    v0 = e.v0;
    v1 = e.v1;
    return true;
}   // end getEdge


// public
size_t ObjModel::findAdjacentFaces( int fid, IntSet& fids) const
{
    assert( _faceIds.count(fid) > 0);
    fids.clear();
    const int* vidxs = fvidxs(fid);

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
const IntSet& ObjModel::getSharedFaces( int vi, int vj) const
{
    if ( _vtxConnectionFaces.count(vi) == 0)
        return EMPTY_INT_SET;
    const unordered_map<int, IntSet>& uicf = _vtxConnectionFaces.at(vi);
    if ( uicf.count(vj) == 0)
        return EMPTY_INT_SET;
    return uicf.at(vj);
}   // end getSharedFaces


// public
int ObjModel::getNumSharedFaces( int vi, int vj) const { return (int)getSharedFaces( vi, vj).size();}


// public
int ObjModel::getNumSharedFaces( int eid) const
{
    int v0, v1;
    if ( !getEdge( eid, v0, v1))
        return 0;
    return getNumSharedFaces( v0, v1);
}   // end getNumSharedFaces


// public
const IntSet& ObjModel::getSharedFaces( int eid) const
{
    int v0, v1;
    if ( !getEdge( eid, v0, v1))
        return EMPTY_INT_SET;
    return getSharedFaces(v0, v1);
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

    // The order of vi,vj is important for ordering vertices on the flipped faces
    // to ensure that normal directions on the flipped faces are the same as those
    // on the original faces.
    poly( f0).getOpposite( vk, vi, vj);

    // f0: i,j,k (i-->l)
    // f1: i,l,j (j-->k)
    // Update vertex to face mappings
    _vtxToFaces.at(vl).insert(f0);
    _vtxToFaces.at(vk).insert(f1);
    _vtxToFaces.at(vi).erase(f0);
    _vtxToFaces.at(vj).erase(f1);

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

    const int eij = edgeId(vi,vj);
    // Update face to edge mappings
    _faceEdgeIdxs[f0].erase(eij);
    _faceEdgeIdxs[f1].erase(eij);
    removeEdge(eij);    // Removes from _edgesToFaces

    int ekl = connectEdge(vk, vl);
    _faceEdgeIdxs[f0].insert(ekl);
    _faceEdgeIdxs[f1].insert(ekl);
    _edgesToFaces[ekl].insert(f0);
    _edgesToFaces[ekl].insert(f1);

    const int eik = edgeId(vi,vk);
    _faceEdgeIdxs[f0].erase(eik);
    _faceEdgeIdxs[f1].insert(eik);
    _edgesToFaces[eik].erase(f0);
    _edgesToFaces[eik].insert(f1);

    const int elj = edgeId(vl,vj);
    _faceEdgeIdxs[f0].insert(elj);
    _faceEdgeIdxs[f1].erase(elj);
    _edgesToFaces[elj].insert(f0);
    _edgesToFaces[elj].erase(f1);

    // Finally, need to reset the vertices in the ObjPolys!
    _faces.at(f0) = ObjPoly( vl, vj, vk);
    _faces.at(f1) = ObjPoly( vi, vl, vk);

    // Remap texture coords if necessary
    const int m0 = faceMaterialId(f0);
    const int m1 = faceMaterialId(f1);
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

        int* f0vtorder = _faces.at(f0).fvindices;       // Vertex order
        cv::Vec3i& f0uvorder = mat.faceUVOrder.at(f0);  // UV order
        int* f1vtorder = _faces.at(f1).fvindices;       // Vertex order
        cv::Vec3i& f1uvorder = mat.faceUVOrder.at(f1);  // UV order
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


namespace {
// Given unit base vector bv (from v) and p (from v), find w as the unit vector perpendicular
// to bv and pointing to p, and return the perpendicular distance from p to the baseline (incident with bv).
double calcBaseNorm( const cv::Vec3d& bv, const cv::Vec3d& p, cv::Vec3d& w)
{
    const cv::Vec3d t = p.dot(bv)*bv;         // Point from p where triangle perpendicular meets base bv.
    const double H = cv::norm(t-p);           // Height of the triangle
    w = H > 0 ? (p-t)/H : cv::Vec3d(0,0,0);   // Unit vector points up from the base perpendicularly
    return H;
}   // end calcBaseNorm

}   // end namespace


// public
cv::Vec3d ObjModel::projectToPoly( int fid, const cv::Vec3d& vx) const
{
    const int* vidxs = fvidxs(fid);
    const cv::Vec3d v0 = vtx( vidxs[0]);

    // Don't assume vx is in the plane of the polygon. First project into the plane.
    const cv::Vec3d z = calcFaceNorm(fid);          // Unit length
    const cv::Vec3d pvx = vx - z.dot(vx - v0)*z;    // Vertex projected into plane of polygon

    // If pvx is now inside the polygon, then we're done: pvx is the projected point.
    if ( isVertexInsideFace( fid, pvx))
        return pvx;

    const cv::Vec3d v1 = vtx( vidxs[1]);
    const cv::Vec3d v2 = vtx( vidxs[2]);

    const cv::Vec3d v10 = v1 - v0;
    const cv::Vec3d v21 = v2 - v1;
    const cv::Vec3d v02 = v0 - v2;
    const double nv10 = cv::norm(v10);
    const double nv21 = cv::norm(v21);
    const double nv02 = cv::norm(v02);
    const cv::Vec3d u = v10 * 1.0/nv10; // v0-->v1
    const cv::Vec3d v = v21 * 1.0/nv21; // v1-->v2
    const cv::Vec3d w = v02 * 1.0/nv02; // v2-->v0

    const cv::Vec3d dv0 = pvx - v0;
    const cv::Vec3d dv1 = pvx - v1;
    const cv::Vec3d dv2 = pvx - v2;

    // Calculate the projected positions along the edges constrained by edge endpoints.
    const double du = std::min( std::max<double>( 0, dv0.dot(u)), nv10);
    const double dv = std::min( std::max<double>( 0, dv1.dot(v)), nv21);
    const double dw = std::min( std::max<double>( 0, dv2.dot(w)), nv02);
    // The corresponding test points of these projections.
    const cv::Vec3d t0 = du*u;
    const cv::Vec3d t1 = dv*v;
    const cv::Vec3d t2 = dw*w;

    const cv::Vec3d* bv = &v0;
    const cv::Vec3d* bt = &t0;
    double bd = l2sq( t0 - dv0);

    double d = l2sq( t1 - dv1);
    if ( d < bd)
    {
        bd = d;
        bv = &v1;
        bt = &t1;
    }   // end if

    d = l2sq( t2 - dv2);
    if ( d < bd)
    {
        bd = d;
        bv = &v2;
        bt = &t2;
    }   // end if

    return *bv + *bt;
}   // end projectToPoly


// public
cv::Vec3d ObjModel::toPropFromAbs( int fid, const cv::Vec3d& vx) const
{
    const int* vidxs = fvidxs(fid);
    const cv::Vec3d v0 = vtx( vidxs[0]);
    const cv::Vec3d v1 = vtx( vidxs[1]);
    const cv::Vec3d v2 = vtx( vidxs[2]);

    const cv::Vec3d vx0 = vx - v0;
    const cv::Vec3d v10 = v1 - v0;
    const cv::Vec3d v21 = v2 - v1;
    const cv::Vec3d v20 = v2 - v0;

    const double nv10 = cv::norm(v10);
    const double nv21 = cv::norm(v21);
    const cv::Vec3d u = nv10 > 0 ? v10 * 1.0/nv10 : cv::Vec3d(0,0,0);
    const cv::Vec3d v = nv21 > 0 ? v21 * 1.0/nv21 : cv::Vec3d(0,0,0);
    const double theta = acos( -u.dot(v));

    cv::Vec3d w;
    const double H = calcBaseNorm( u, v20, w);

    const double h = w.dot(vx0 - vx0.dot(u)*u); // Use dot product here to get sign of height (what side of triangle base)
    // By similar triangles the edge parallel to v2v1 when shifted to be incident with vx makes angle theta with edge v1v0.
    const double b = fabs(theta) > 0 ? h/sin(theta) : 0;
    const double a = u.dot(vx0 - b*v); // Sign of b ensures that vx0 - b*v is always incident with base v0,v1

    const double xprop = nv10 > 0 ? a/nv10 : 0;
    const double yprop = nv21 > 0 ? b/nv21 : 0;

    const cv::Vec3d z = calcFaceNorm(fid);  // Unit length
    const double A = H*nv10;                // This is twice the area of the triangle
    const double zdist = z.dot(vx0);        // Projected distance off the triangle's surface in direction of normal
    const double zprop = A > 0 ? zdist / sqrt(A) : 0; // sqrt because distance needs to scale linearly (obviously)
    return cv::Vec3d( xprop, yprop, zprop);
}   // end toPropFromAbs


// public
cv::Vec3d ObjModel::toAbsFromProp( int fid, const cv::Vec3d& vp) const
{
    const int* vidxs = fvidxs(fid);
    const cv::Vec3d v0 = vtx( vidxs[0]);
    const cv::Vec3d v1 = vtx( vidxs[1]);
    const cv::Vec3d v2 = vtx( vidxs[2]);
    const double A = calcTriangleArea( v0, v1, v2);
    const cv::Vec3d z = calcFaceNorm(fid);  // Unit length
    return v0 + vp[0]*(v1-v0) + vp[1]*(v2-v1) + vp[2]*sqrt(2*A)*z;
}   // end toAbsFromProp


// public
bool ObjModel::isVertexInsideFace( int fid, const cv::Vec3d& x) const
{
    const int* vidxs = fvidxs( fid);
    const cv::Vec3d va = vtx( vidxs[0]);
    const cv::Vec3d vb = vtx( vidxs[1]);
    const cv::Vec3d vc = vtx( vidxs[2]);

    const double a = cv::norm( vb - vc);
    const double b = cv::norm( va - vc);
    const double c = cv::norm( va - vb);
    const double A = calcTriangleArea(a,b,c) + pow(10, -_fltPrc);   // Area of whole triangle plus slight allowance

    const double ia = cv::norm( va - x);
    const double ib = cv::norm( vb - x);
    const double ic = cv::norm( vc - x);

    return (calcTriangleArea( a,ib,ic) + calcTriangleArea( b,ia,ic) + calcTriangleArea( c,ia,ib)) <= A;
}   // end isVertexInsideFace


// public
const IntSet& ObjModel::getFaceIds( int vi) const
{
    if ( _vtxToFaces.count(vi) == 0)
        return EMPTY_INT_SET;
    return _vtxToFaces.at(vi);
}   // end getFaceIds


// public
int ObjModel::lookupVertexIndex( const cv::Vec3f& v) const
{
    Key3L key = toKey( v[0], v[1], v[2], _fltPrc);
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
        for ( int vid : vids)
        {
            const cv::Vec3f& v = getVertex( vid);
            std::cerr << "\tVTX_" << vid << "): x=" << v[0] << ", y=" << v[1] << ", z=" << v[2] << " [VTX connections:";
            // Show connected vertices
            const IntSet& cvs = getConnectedVertices( vid);
            std::for_each( std::begin(cvs), std::end(cvs), [](int cv){ std::cerr << " " << cv;});
            std::cerr << "]" << std::endl;
        }   // end for
        std::cerr << std::endl;
    }   // end if

    const IntSet& matIds = getMaterialIds();
    std::cerr << " Model has " << matIds.size() << " materials" << std::endl;
    for ( int matId : matIds)
    {
        const Material& mat = *_materials.at(matId);
        std::cerr << " Material " << matId << " has [" << mat.ambient.size() << "," << mat.diffuse.size() << "," << mat.specular.size()
                  << "] ambient,diffuse,specular maps" << std::endl;
        std::cerr << " Material has " << mat.uvs.size() << " UV coordinates referencing it" << std::endl;
    }   // end foreach

    if ( withDetails)
    {
        std::cerr << " Face vertex & UV indices:" << std::endl;
        for ( int fid : fids)
        {
            const int* vids = fvidxs(fid);
            std::cerr << "    F_" << fid << ") VTs: " << vids[0] << ", " << vids[1] << ", " << vids[2] << std::endl;
            const int mid = faceMaterialId(fid);
            if ( mid >= 0)
            {
                const int* uvis = faceUVs(fid);
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
    std::cerr << " C _fCounter =                " << std::setw(8) << _fCounter << std::endl;
    std::cerr << " C _eCounter =                " << std::setw(8) << _eCounter << std::endl;
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
