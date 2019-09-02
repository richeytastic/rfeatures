/************************************************************************
 * Copyright (C) 2019 Richard Palmer
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
#include <FeatureUtils.h>
using RFeatures::ObjModel;
using RFeatures::ObjPoly;
using RFeatures::ObjEdge;
using RFeatures::ObjMaterial;


namespace {
static const IntSet EMPTY_INT_SET;
static const size_t HASH_NDP = 4;
}   // end namespace


size_t ObjModel::numTextureEdges( int va, int vb) const
{
    const IntSet& sfids = spolys( va, vb);
    if ( sfids.empty())
        return 0;

    size_t ntex = 0;
    IntSet tset;    // Records the texture vertices
    for ( int fid : sfids)
    {
        const int* uvs = faceUVs( fid);
        if ( !uvs)
            continue;

        const ObjPoly& fc = face(fid);

        int a = 0;  // First edge vertex index
        int b = 1;  // Second edge vertex index
        if ( fc[0] == va)
        {
            a = 0;
            b = fc[1] == vb ? 1 : 2;
        }   // end if
        else if ( fc[1] == va)
        {
            a = 1;
            b = fc[0] == vb ? 0 : 2;
        }   // end else if
        else
        {
            a = 2;
            b = fc[0] == vb ? 0 : 1;
        }   // end else if

        assert( fc[a] == va);
        assert( fc[b] == vb);

        // fc[a] == va and fc[b] == vb, so uvs[a] and uvs[b] is the corresponding texture edge.
        // If either of the texture vertices are new, it counts as a new texture edge.
        if ( tset.count(uvs[a]) == 0 || tset.count(uvs[b]) == 0)
            ntex++;
        tset.insert(uvs[a]);
        tset.insert(uvs[b]);
    }   // end for

    return ntex;
}   // end numTextureEdges


ObjModel::Ptr ObjModel::deepCopy( bool shareMats) const
{
    ObjModel* m = new ObjModel(*this);
    if ( !shareMats)
    {
        for ( auto& p : m->_mats)
            p.second._tx = p.second._tx.clone();
    }   // end for
    return Ptr( m, [](ObjModel* x){delete x;});
}   // end deepCopy


ObjModel::Ptr ObjModel::repackedCopy( bool shareMats) const
{
    ObjModel* m = new ObjModel;
    m->_tmat = _tmat;
    m->_imat = _imat;

    std::unordered_map<int,int> vvmap;  // Old to new vertex IDs
    for ( int vid : _vids)
        vvmap[vid] = m->addVertex( _vtxs.at(vid));

    std::unordered_map<int,int> ffmap;  // Old to new face IDs
    for ( int fid : _fids)
    {
        const int* f = fvidxs(fid);
        ffmap[fid] = m->addFace( vvmap[f[0]], vvmap[f[1]], vvmap[f[2]]);
    }   // end for

    m->copyInMaterials( *this, shareMats);   // Note that material IDs don't need to be sequential
    for ( const auto& p : _f2m)
    {
        const cv::Vec2f& uv0 = faceUV(p.first, 0);
        const cv::Vec2f& uv1 = faceUV(p.first, 1);
        const cv::Vec2f& uv2 = faceUV(p.first, 2);
        m->setOrderedFaceUVs( p.second, ffmap[p.first], uv0, uv1, uv2);
    }   // end for

    return Ptr( m, [](ObjModel* x){delete x;});
}   // end repackedCopy


// public static
ObjModel::Ptr ObjModel::create()
{
    return Ptr( new ObjModel, [](ObjModel* x){delete x;});
}   // end create


// private
ObjModel::ObjModel() :  _vCounter(0), _fCounter(0), _eCounter(0), _mCounter(0), _tmat(cv::Matx44d::eye()) {}


// private
ObjModel::~ObjModel() {}


void ObjModel::setTransformMatrix( const cv::Matx44d& tmat)
{
    _tvtxs.clear(); // Will force recalculations of vertex positions
    _tmat = tmat;
    _imat = tmat.inv();
}   // end setTransformMatrix


void ObjModel::addTransformMatrix( const cv::Matx44d& tmat) { setTransformMatrix( tmat * _tmat);}


void ObjModel::fixTransformMatrix()
{
    for ( int vidx : _vids)
        _vtxs[vidx] = vtx(vidx); // Ensures transform against existing matrix is done.
    _tmat = cv::Matx44d::eye(); // NB not necessary to clear vertex cache _tvtxs
    _imat = _tmat;
}   // end fixTransformMatrix


const cv::Vec3f& ObjModel::vtx( int vidx) const
{
    if ( _tvtxs.count(vidx) == 0)
    {
        assert( _vtxs.count(vidx) > 0);
        _tvtxs[vidx] = transform( _tmat, _vtxs.at(vidx)); // _tvtxs is mutable
    }   // end if
    return _tvtxs.at(vidx);
}   // end vtx


int ObjModel::faceMaterialId( int fid) const
{
    assert( _fids.count(fid) > 0);
    if ( _f2m.count(fid) == 0) // No material for this face
        return -1;
    return _f2m.at(fid);
}   // end faceMaterialId


const IntSet& ObjModel::materialFaceIds( int mid) const
{
    if ( _mats.count(mid) > 0)
        return _mats.at(mid)._fids;
    return EMPTY_INT_SET;
}   // end materialFaceIds


int ObjModel::addMaterial( const cv::Mat& m, size_t maxd)
{
    if ( m.empty())
        return -1;
    const int mid = _mCounter++;
    _addMaterial( mid, m, maxd);
    return mid;
}   // end addMaterial


void ObjModel::_addMaterial( int mid, const cv::Mat& m, size_t maxd)
{
    _mids.insert( mid);
    _mats[mid]._tx = shrinkMax( m, maxd);
}   // end _addMaterial


void ObjModel::removeMaterial( int mid)
{
    _mids.erase(mid);
    _mats.erase(mid);
}   // end removeMaterial


void ObjModel::removeAllMaterials()
{
    while ( !_mats.empty())
        removeMaterial( _mats.begin()->first);
    _mCounter = 0;
}   // end removeAllMaterials


void ObjModel::copyInMaterials( const ObjModel& omc, bool shareMats)
{
    const IntSet& matIds = omc.materialIds();
    for ( int mid : matIds)
    {
        const cv::Mat& tx = omc.texture(mid);
        _addMaterial( mid, shareMats ? tx : tx.clone(), size_t(std::max(tx.rows, tx.cols)));
    }   // end for
    _mCounter = omc._mCounter;
}   // end copyInMaterials


namespace {
// Calculate a new UV coordinate from an old UV.
void calcNewUV( cv::Vec2f& uv, int nrows, int ncols, const std::vector<int>& scols, int i)
{
    // v is unchanged, only u affected
    const float oldWidth = static_cast<float>((i == (int)(scols.size())-1) ? ncols - scols[i] : scols[i+1] - scols[i]);
    uv[0] = (scols[i] + (uv[0] * oldWidth))/ncols;
}   // end calcNewUV
}   // end namespace


size_t ObjModel::mergeMaterials()
{
    if ( _mats.size() <= 1)
        return _mats.size();

    const IntSet mids = materialIds();   // Copied out because changing
    const int nmats = (int)mids.size();

    // Concatenate images across all of the materials into a single texture image.
    std::vector<cv::Mat> txs; // Textures from all materials
    std::vector<int> midSeq;  // Repeatable sequence of material IDs
    for ( int mid : mids)
    {
        midSeq.push_back(mid);
        cv::Mat tx  = texture(mid);
        if ( !tx.empty())
            txs.push_back(tx);
    }   // end for

    std::vector<int> scols; // The starting columns for the concatenated texture images
    const cv::Mat tx = concatHorizontalMax( txs, &scols);
    const int nrows = tx.rows;
    const int ncols = tx.cols;

    const int mmid = addMaterial( tx); // Create the new "merge" material

    for ( int i = 0; i < nmats; ++i)
    {
        const int mid = midSeq[i];
        // Map all the faces from material m to mmat
        const IntSet& fids = materialFaceIds( mid);
        assert( !fids.empty());
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
        }   // end for

        removeMaterial( mid);   // Remove the old material
    }   // end for

    return nmats;   // The number of materials that were merged.
}   // end mergeMaterials



const cv::Mat& ObjModel::texture( int mid) const
{
    assert( _mats.count(mid) > 0);
    return _mats.at(mid)._tx;
}   // end texture


void ObjModel::_removeFaceUVs( int mid, int fid)
{
    assert( materialIds().count( mid) > 0);
    _mats.at(mid).removeFaceUVs(fid);
    _f2m.erase(fid);
}   // end _removeFaceUVs


int ObjModel::addVertex( float x, float y, float z)
{
    return addVertex( cv::Vec3f(x,y,z));
}   // end addVertex


int ObjModel::addVertex( const cv::Vec3f& v)
{
#ifndef NDEBUG
    if ( _tmat != cv::Matx44d::eye())
        std::cerr << "[WARNING] RFeatures::ObjModel::addVertex: Adding vertex to model after setting the transform matrix!" << std::endl;
#endif

    if ( cvIsNaN( v[0]) || cvIsNaN( v[1]) || cvIsNaN( v[2]))
    {
#ifndef NDEBUG
        std::cerr << "[WARNING] RFeatures::ObjModel::addVertex: Added vertex has at least one NaN entry!" << std::endl;
#endif
        return -1;
    }   // end if

    size_t key = hash( v, HASH_NDP);
    if ( _v2id.count( key) > 0)
        return _v2id.at(key);

    const int vi = _vCounter++;

    _v2id[key] = vi;
    _vids.insert(vi);
    _vtxs[vi] = v;

    return vi;
}   // end addVertex


bool ObjModel::removeVertex( int vi)
{
    assert( _vids.count(vi) > 0);
    assert(_v2v.count(vi) == 0);
    assert(_v2e.count(vi) == 0);
    assert(_v2f.count(vi) == 0);

    // Check that this vertex is no longer connected to others by edges/faces.
    const IntSet& cvs = cvtxs(vi);
    if ( !cvs.empty())
        return false;

    const cv::Vec3f& v = _vtxs.at(vi);
    size_t key = hash(v, HASH_NDP);
    _v2id.erase(key);
    _vids.erase(vi);
    _vtxs.erase(vi);
    _tvtxs.erase(vi);

    return true;
}   // end removeVertex


cv::Vec2f ObjModel::calcTextureCoords( int fidx, const cv::Vec3f& p) const
{
    const int matId = faceMaterialId( fidx);
    if ( matId < 0)
        return cv::Vec2f(-1,-1);

    const int* vidxs = fvidxs(fidx);
    const cv::Vec3f& v0 = vtx(vidxs[0]);
    const cv::Vec3f& v1 = vtx(vidxs[1]);
    const cv::Vec3f& v2 = vtx(vidxs[2]);

    const double garea = calcTriangleArea(v0,v1,v2);    // Geometry area
    const double C = cv::norm(v2-v1);   // Picture C as base of triangle with v2 bottom left, v1 bottom right (v0 at top).

    const double Y = cv::norm(p-v1);
    const double Z = cv::norm(p-v2);

    // Calculate G: the scaling constant for the triangle areas.
    const int* uvs = faceUVs( fidx);
    const cv::Vec2d t0 = uv( matId, uvs[0]);
    const cv::Vec2d t1 = uv( matId, uvs[1]);
    const cv::Vec2d t2 = uv( matId, uvs[2]);
    const double AT = cv::norm(t2-t0);
    const double BT = cv::norm(t1-t0);
    const double CT = cv::norm(t2-t1);

    const double tarea = calcTriangleArea(AT,BT,CT);    // Texture area

    double G = 1.0;
    if ( garea > 0.0)
        G = tarea / garea;
    else if ( tarea > 0.0)
        G = 0.0;

    double b = 0.0;
    if ( C > 0.0)
        b = 2.0*calcTriangleArea(Y,Z,C)/C;   // b is height of triangle YZC and is at right angles to line segment C

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

    cv::Vec2d unitVecC; // Direction vector along texture triangle line segment.
    cv::normalize( t2-t1, unitVecC);    // Use the OpenCV function to account for possibility of CT == 0.0
    // unitVecC needs scaling by CTsubLen and an orthogonal vector (toward t0) scaled to length bT added to
    // give the texture coordinates with respect to texture vertex t1.
    // Depending on texture mapping, either unitVecB or unitVecB2 will be the correct orthogonal vector to unitVecC.
    // Test by seeing which (when added to t1) is closer to t0.
    cv::Vec2d unitVecB( -unitVecC[1], unitVecC[0]);
    const cv::Vec2d unitVecB2( unitVecC[1], -unitVecC[0]);
    const double delta0 = l2sq( t1 + unitVecB - t0);
    const double delta1 = l2sq( t1 + unitVecB2 - t0);
    if ( delta1 < delta0)
        unitVecB = unitVecB2;

    const cv::Vec2d ov = t1 + (CTsubLen * unitVecC) + (bT * unitVecB);
    assert( !cvIsNaN( ov[0]) && !cvIsNaN( ov[1]));
    return cv::Vec2f( static_cast<float>(ov[0]), static_cast<float>(ov[1]));
}   // end calcTextureCoords


bool ObjModel::adjustVertex( int vidx, float x, float y, float z)
{
    return adjustVertex( vidx, cv::Vec3f(x,y,z));
}   // end adjustVertex


bool ObjModel::adjustVertex( int vidx, const cv::Vec3f& v)
{
    if ( cvIsNaN( v[0]) || cvIsNaN( v[1]) || cvIsNaN( v[2]))
        return false;

    if ( _vids.count(vidx) == 0)
        return false;

    cv::Vec3f& vec = _vtxs.at(vidx);   // The vertex to modify
    const size_t h = hash(vec, HASH_NDP);
    assert( _v2id.count(h) > 0);
    _v2id.erase( h); // Remove original vertex hash value
    vec = v; // Update with new position of vertex
    _v2id[ hash(vec, HASH_NDP)] = vidx;  // Hash back with new vertices
    _tvtxs.erase(vidx); // Force recalculation of cached vertex

    return true;
}   // end adjustVertex


bool ObjModel::scaleVertex( int vidx, float sf)
{
    if ( cvIsNaN( sf))
        return false;

    if ( _vids.count(vidx) == 0)
        return false;

    cv::Vec3f& vec = _vtxs.at(vidx);   // The vertex to modify
    const size_t h = hash(vec, HASH_NDP);
    assert( _v2id.count(h) > 0);
    _v2id.erase( h);    // Remove original vertex hash value
    vec = sf*vec; // Update with new position of vertex
    _v2id[ hash( vec, HASH_NDP)] = vidx;  // Hash back with new vertices
    _tvtxs.erase(vidx); // Force recalculation of cached vertex

    return true;
}   // end scaleVertex


bool ObjModel::hasEdge( int v0, int v1) const
{
    return _v2v.count(v0) > 0 ? _v2v.at(v0).count(v1) > 0 : false;
}   // end hasEdge


int ObjModel::edgeId( int v0, int v1) const
{
    const ObjEdge ledge(v0,v1); // Lookup edge
    if ( _e2id.count(ledge) == 0)
        return -1;
    return _e2id.at(ledge);
}   // end edgeId


int ObjModel::_connectEdge( int v0, int v1)
{
    assert( v0 != v1);
    if ( hasEdge(v0,v1))
        return edgeId(v0,v1);
    const int eidx = _eCounter++;
    _connectEdge( eidx, v0, v1);
    return eidx;
}   // end _connectEdge


void ObjModel::_connectEdge( int ei, int v0, int v1)
{
    const ObjEdge& e = _edges[ei] = ObjEdge( v0, v1);  // e[0] < e[1]
    _e2id[e] = ei;  // Reverse lookup
    _eids.insert(ei);
    _v2v[v0].insert(v1);
    _v2v[v1].insert(v0);
    _v2e[v0].insert(ei);
    _v2e[v1].insert(ei);
}   // end _connectEdge


void ObjModel::_removeEdge( int ei)
{
    const ObjEdge& e = _edges[ei];

    _v2v[e[0]].erase(e[1]);
    if ( _v2v[e[0]].empty())
        _v2v.erase(e[0]);

    _v2v[e[1]].erase(e[0]);
    if ( _v2v[e[1]].empty())
        _v2v.erase(e[1]);

    _v2e[e[0]].erase(ei);
    if ( _v2e[e[0]].empty())
        _v2e.erase(e[0]);

    _v2e[e[1]].erase(ei);
    if ( _v2e[e[1]].empty())
        _v2e.erase(e[1]);

    if ( _e2f[ei].empty())
        _e2f.erase(ei);

    _e2id.erase(e);
    _eids.erase(ei);
    _edges.erase(ei);
}   // end _removeEdge


int ObjModel::addEdge( int v0, int v1)
{
    if ( hasEdge(v0,v1))
        return edgeId(v0,v1);

    // If there are vertices T currently connected to v0 that are also connected to v1,
    // then connecting v0 and v1 directly will create new polygons between the
    // currently connected vertices T and v0 and v1.
    const IntSet& u0Conn = cvtxs( v0);
    const IntSet& u1Conn = cvtxs( v1);
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

    const int ei = _connectEdge( v0, v1);
    // Create the polygon(s)
    for ( int v : ucs)
        addFace( v, v0, v1);
    return ei;
}   // end addEdge


int ObjModel::face( int v0, int v1, int v2) const
{
    if ( hasEdge(v0,v1) && hasEdge(v1,v2) && hasEdge(v2,v0))
    {
        const int e01 = edgeId(v0,v1);
        const int e02 = edgeId(v0,v2);
        const IntSet& fs0 = _e2f.at(e01);
        const IntSet& fs1 = _e2f.at(e02);

        // Look for the common face by iterating over the smaller set
        if ( fs0.size() < fs1.size())
        {
            for ( int fc : fs0)
            {
                if ( fs1.count(fc) > 0)
                    return fc;
            }   // end for
        }   // end if
        else
        {
            for ( int fc : fs1)
            {
                if ( fs0.count(fc) > 0)
                    return fc;
            }   // end for
        }   // end else
    }   // end if

    return -1;
}   // end face


int ObjModel::addFace( const int* vtxs) { return addFace( vtxs[0], vtxs[1], vtxs[2]);}


int ObjModel::addFace( int v0, int v1, int v2)
{
    assert( _vids.count(v0) > 0);
    assert( _vids.count(v1) > 0);
    assert( _vids.count(v2) > 0);

    const ObjPoly fc( v0, v1, v2); // Don't add if already present
    if ( _f2id.count(fc) > 0)
        return _f2id.at(fc);

    const int fid = _fCounter++;
    _faces[fid] = fc;
    _fids.insert(fid);
    _f2id[fc] = fid;

    _v2f[v0].insert(fid);
    _v2f[v1].insert(fid);
    _v2f[v2].insert(fid);

    int e01 = _connectEdge( v0, v1);
    int e12 = _connectEdge( v1, v2);
    int e20 = _connectEdge( v2, v0);

    _e2f[e01].insert(fid);
    _e2f[e12].insert(fid);
    _e2f[e20].insert(fid);

    return fid;
}   // end addFace


bool ObjModel::removePoly( int fid)
{
    assert( _fids.count(fid) > 0);
    if ( _fids.count(fid) == 0)
        return false;

    // Remove from the Material if present
    const int matid = faceMaterialId( fid);
    if ( matid >= 0)
        _removeFaceUVs( matid, fid);

    const int* vidxs = fvidxs(fid);
    const int v0 = vidxs[0];
    const int v1 = vidxs[1];
    const int v2 = vidxs[2];

    _v2f[v0].erase(fid);
    if ( _v2f[v0].empty())
        _v2f.erase(v0);

    _v2f[v1].erase(fid);
    if ( _v2f[v1].empty())
        _v2f.erase(v1);

    _v2f[v2].erase(fid);
    if ( _v2f[v2].empty())
        _v2f.erase(v2);

    const int e01 = edgeId(v0,v1);
    const int e12 = edgeId(v1,v2);
    const int e20 = edgeId(v2,v0);

    // Remove the entries if the connection between vertices shares no more faces.
    _e2f[e01].erase(fid);
    if ( _e2f[e01].empty())
    {
        _e2f[e01].erase(fid);
        _removeEdge(e01);
    }   // end if

    _e2f[e12].erase(fid);
    if ( _e2f[e12].empty())
    {
        _e2f[e12].erase(fid);
        _removeEdge(e12);
    }   // end if

    _e2f[e20].erase(fid);
    if ( _e2f[e20].empty())
    {
        _e2f[e20].erase(fid);
        _removeEdge(e20);
    }   // end if

    _f2id.erase(_faces[fid]);
    _fids.erase(fid);
    _faces.erase(fid);

    return true;
}   // end removeFace


bool ObjModel::setOrderedFaceUVs( int mid, int fid, const cv::Vec2f& uv0, const cv::Vec2f& uv1, const cv::Vec2f& uv2)
{
    assert( _fids.count(fid) > 0);
    if ( _fids.count(fid) == 0)
        return false;
    assert( _mids.count(mid) > 0);
    if ( _mats.count(mid) == 0)
        return false;

    ObjMaterial& mat = _mats[mid];

    assert( !cvIsNaN( uv0[0]) && !cvIsNaN( uv0[1]));
    assert( !cvIsNaN( uv1[0]) && !cvIsNaN( uv1[1]));
    assert( !cvIsNaN( uv2[0]) && !cvIsNaN( uv2[1]));
    assert( mat._f2uv.count(fid) == 0); // Can't have added previously to material

    _f2m[fid] = mid;    // Overwrite material lookup for poly.
    mat.mapFaceUVs( fid, uv0, uv1, uv2);

    return true;
}   // end setOrderedFaceUVs


bool ObjModel::setOrderedFaceUVs( int mid, int fid, const cv::Vec2f uvs[3])
{
    return setOrderedFaceUVs( mid, fid, uvs[0], uvs[1], uvs[2]);
}   // end setOrderedFaceUVs


const IntSet& ObjModel::uvs( int mid) const
{
    assert( _mats.count(mid) > 0);
    return _mats.at(mid)._uvIds;
}   // end uvs


const cv::Vec2f& ObjModel::uv( int mid, int uvID) const
{
    assert( _mats.count(mid) > 0);
    const ObjMaterial& mat = _mats.at(mid);
    assert( mat._uvs.count(uvID) > 0);
    return mat._uvs.at(uvID);
}   // end uv


const cv::Vec2f& ObjModel::faceUV( int fid, int i) const
{
    assert( _fids.count(fid) > 0);
    assert( i >= 0 && i < 3);
    const int mid = faceMaterialId(fid);
    assert( mid >= 0);
    const ObjMaterial& mat = _mats.at(mid);
    return mat._uvs.at( mat._f2uv.at(fid)[i]);
}   // end faceUV


const int* ObjModel::faceUVs( int fid) const
{
    assert( _fids.count(fid) > 0);
    const int mid = faceMaterialId(fid);
    if ( mid < 0)
        return nullptr;
    const ObjMaterial& mat = _mats.at(mid);
    return &mat._f2uv.at(fid)[0];
}   // end faceUVs


const int* ObjModel::fvidxs( int fid) const
{
    assert( _fids.count(fid) > 0);
    if ( _fids.count(fid) == 0)
        return nullptr;
    return _faces.at(fid).vertices();
}   // end fvidxs


void ObjModel::reversePolyVertices( int fid)
{
    assert( _fids.count(fid) > 0);
    ObjPoly& fc = _faces.at(fid);
    std::swap( fc[0], fc[2]);

    const int mid = faceMaterialId( fid);
    if ( mid >= 0)
    {
        cv::Vec3i& fuvis = _mats.at(mid)._f2uv.at(fid);
        std::swap( fuvis[0], fuvis[2]);
    }   // end if
}   // end reversePolyVertices


namespace {
cv::Vec3f normaliseCross( const cv::Vec3f& vi, const cv::Vec3f& vj)
{
    cv::Vec3f fn;
    cv::normalize( vi.cross(vj), fn);
    return fn;
}   // end normaliseCross
}   // end namespace


cv::Vec3f ObjModel::calcFaceNorm( int fid) const
{
    assert( _fids.count(fid) > 0);
    const ObjPoly& f = _faces.at(fid);
    return normaliseCross( vtx(f[1]) - vtx(f[0]), vtx(f[2]) - vtx(f[1]));
}   // end calcFaceNorm


cv::Vec3f ObjModel::calcFaceNorm( int fid, cv::Vec3f& vi, cv::Vec3f& vj) const
{
    const int* vidxs = fvidxs(fid);
    assert(vidxs);
    cv::normalize( vtx(vidxs[1]) - vtx(vidxs[0]), vi);
    cv::normalize( vtx(vidxs[2]) - vtx(vidxs[1]), vj);
    return normaliseCross( vi, vj);
}   // end calcFaceNorm


double ObjModel::calcFaceArea( int fid) const
{
    const int* vidxs = fvidxs(fid);
    assert(vidxs);
    return calcTriangleArea( _vtxs.at(vidxs[0]), _vtxs.at(vidxs[1]), _vtxs.at(vidxs[2]));
}   // end calcFaceArea


bool ObjModel::removeEdge( int ei)
{
    if ( _edges.count(ei) == 0)
        return false;

    const ObjEdge& e = edge( ei);
    // Removing an edge removes all attached faces
    const IntSet fids = spolys( e[0], e[1]);    // Copy out
    for ( int fid : fids)
        removePoly( fid);   // May remove edge if shared by a single face
    if ( _edges.count(ei) > 0)
        _removeEdge( ei);
    return true;
}   // end removeEdge


bool ObjModel::removeEdge( int vi, int vj) { return removeEdge( edgeId( vi, vj));}

/*
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
    const IntSet& sfids = spolys( vi, vj);
    if ( sfids.empty())
        return false;

    assert( vn >= 0 && vn != vi && vn != vj);
    for ( int fid : sfids)
    {
        const int* vidxs = fvidxs(fid);

        // Create two new faces
        // Order of vidxs will be clockwise and match the order of uvs.
        // This order needs to be maintained so that polygon normals don't flip.
        const int vk = face(fid).opposite(vi,vj);    // Vertex on the shared face that isn't the edge vertex
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
    _removeEdge( eid); // Finally, remove the old polygons attached to the edge
    return true;
}   // end subDivideEdge
*/


bool ObjModel::edge( int ei, int& v0, int& v1) const
{
    if ( _edges.count(ei) == 0)
        return false;
    const ObjEdge& e = edge(ei);
    v0 = e[0];
    v1 = e[1];
    return true;
}   // end edge


const IntSet& ObjModel::spolys( int vi, int vj) const
{
    if ( !hasEdge(vi,vj))
        return EMPTY_INT_SET;
    return spolys( edgeId(vi,vj));
}   // end spolys 


int ObjModel::nspolys( int vi, int vj) const { return static_cast<int>( spolys( vi, vj).size());}


int ObjModel::nspolys( int eid) const
{
    int v0, v1;
    if ( !edge( eid, v0, v1))
        return 0;
    return nspolys( v0, v1);
}   // end nspolys


const IntSet& ObjModel::spolys( int ei) const
{
    assert( _eids.count(ei) > 0);
    if ( _e2f.at(ei).empty())
        return EMPTY_INT_SET;
    return _e2f.at(ei);
}   // end spolys


bool ObjModel::flipFacePair( int vi, int vj)
{
    const IntSet& sfids = spolys( vi, vj);
    if ( sfids.size() <= 1 || sfids.size() > 2)
        return false;

    int fid0 = *sfids.begin();
    int fid1 = *(++sfids.begin());

    ObjPoly& f0 = _faces[fid0];
    ObjPoly& f1 = _faces[fid1];
    const int vk = f0.opposite( vi, vj);
    const int vl = f1.opposite( vi, vj);

    // The order of vi,vj is important for ordering vertices on the flipped faces
    // to ensure that normal directions on the flipped faces are the same as those
    // on the original faces.
    f0.opposite( vk, vi, vj);

    // Update vertex to face mappings
    // f0: i,j,k (i-->l)
    // f1: i,l,j (j-->k)

    // vi and vj no longer connected
    _v2f[vi].erase(fid0);
    _v2f[vj].erase(fid1);
    const int eij = edgeId(vi,vj);
    _e2f[eij].erase(fid0);
    _e2f[eij].erase(fid1);
    _removeEdge(eij);

    // vk and vl now connect
    const int ekl = _connectEdge(vk, vl);
    _v2f[vl].insert(fid0);
    _v2f[vk].insert(fid1);
    _e2f[ekl].insert(fid0);
    _e2f[ekl].insert(fid1);

    const int eik = edgeId(vi,vk);
    _e2f[eik].erase(fid0);
    _e2f[eik].insert(fid1);

    const int ejl = edgeId(vj,vl);
    _e2f[ejl].erase(fid1);
    _e2f[ejl].insert(fid0);

    // Reset the vertices in the ObjPolys
    f0[0] = vl;
    f0[1] = vj;
    f0[2] = vk;
    f1[0] = vi;
    f1[1] = vl;
    f1[2] = vk;

    // Remap texture coords if necessary
    const int m0 = faceMaterialId(fid0);
    const int m1 = faceMaterialId(fid1);
    if ( m0 != m1)
    {
        // Remove texture coords if not the same!
        if (m0 >= 0)
            _removeFaceUVs(m0, fid0);
        if (m1 >= 0)
            _removeFaceUVs(m1, fid1);
    }   // end if
    else if ( m0 >= 0)
    {   
        // Faces share the same material ID (m0 or m1)
        ObjMaterial& mat = _mats.at(m0);

        int* f0vtorder = f0._fvindices;      // Vertex order
        cv::Vec3i& f0uvorder = mat._f2uv.at(fid0);  // UV order
        int* f1vtorder = f1._fvindices;      // Vertex order
        cv::Vec3i& f1uvorder = mat._f2uv.at(fid1);  // UV order
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

        mat._uv2f[uvi].erase(fid0);
        mat._uv2f[uvj].erase(fid1);
        mat._uv2f[uvk].insert(fid1);
        mat._uv2f[uvl].insert(fid0);

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
    const cv::Vec3d t = p.dot(bv)*bv;        // Point from p where triangle perpendicular meets base bv.
    const double H = cv::norm(t-p);           // Height of the triangle
    w = H > 0 ? (p-t)/H : cv::Vec3d(0,0,0);  // Unit vector points up from the base perpendicularly
    return H;
}   // end calcBaseNorm
}   // end namespace


cv::Vec3d ObjModel::projectToPoly( int fid, const cv::Vec3d& vx) const
{
    const int* vidxs = fvidxs(fid);
    const cv::Vec3d v0 = vtx( vidxs[0]);
    const cv::Vec3d v1 = vtx( vidxs[1]);
    const cv::Vec3d v2 = vtx( vidxs[2]);

    // Don't assume vx is in the plane of the polygon. First project into the plane.
    const cv::Vec3d z = calcFaceNorm(fid);          // Unit length
    const cv::Vec3d pvx0 = vx - z.dot(vx - v0)*z;    // Vertex projected into plane of polygon
    const cv::Vec3d pvx1 = vx - z.dot(vx - v1)*z;    // Vertex projected into plane of polygon
    const cv::Vec3d pvx2 = vx - z.dot(vx - v2)*z;    // Vertex projected into plane of polygon

    cv::Vec3d pvx = (pvx0 + pvx1 + pvx2) * 1.0/3;

    // If pvx is now inside the polygon, then we're done: pvx is the projected point.
    if ( isVertexInsideFace( fid, pvx))
        return pvx;

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
    const cv::Vec3d tu = du*u;
    const cv::Vec3d tv = dv*v;
    const cv::Vec3d tw = dw*w;

    const cv::Vec3d pv0d = tu - dv0;    // Orthogonal vector that drops to u baseline
    const cv::Vec3d pv1d = tv - dv1;    // Orthogonal vector that drops to v baseline
    const cv::Vec3d pv2d = tw - dv2;    // Orthogonal vector that drops to w baseline

    // Get the lengths of these lines for comparison
    double bd = l2sq( pv0d);
    const double d1 = l2sq( pv1d);
    const double d2 = l2sq( pv2d);

    const cv::Vec3d* bv = &v0;
    const cv::Vec3d* bt = &tu;
    if ( d1 < bd)
    {
        bd = d1;
        bv = &v1;
        bt = &tv;
    }   // end if

    if ( d2 < bd)
    {
        bv = &v2;
        bt = &tw;
    }   // end if

    return *bv + *bt;
}   // end projectToPoly


bool ObjModel::isVertexInsideFace( int fid, const cv::Vec3d& x) const
{
    const int* vidxs = fvidxs( fid);
    const cv::Vec3d va = vtx( vidxs[0]);
    const cv::Vec3d vb = vtx( vidxs[1]);
    const cv::Vec3d vc = vtx( vidxs[2]);

    const double a = cv::norm( vb - vc);
    const double b = cv::norm( va - vc);
    const double c = cv::norm( va - vb);
    const double A = calcTriangleArea( a, b, c);

    const double ia = cv::norm( x - va);
    const double ib = cv::norm( x - vb);
    const double ic = cv::norm( x - vc);

    const double T0 = calcTriangleArea( a,ib,ic);
    const double T1 = calcTriangleArea( b,ia,ic);
    const double T2 = calcTriangleArea( c,ia,ib);
    const double Tsum = T0 + T1 + T2;

    // If the sum of the areas of the three individual triangles is *appreciably* more than
    // the area of the original triangle, then the vertex is outside of the triangle.
    static const size_t NDP = 5;
    const double areaDiff = roundndp( Tsum - A, NDP);
    // Note here that areaDiff cannot be negative (unless choosing too large NDP causes overflow).
    assert( areaDiff >= 0);
    return areaDiff == 0.0;
}   // end isVertexInsideFace


cv::Vec3f ObjModel::toPropFromAbs( int fid, const cv::Vec3f& vx) const
{
    const int* vidxs = fvidxs(fid);
    const cv::Vec3f& v0 = vtx( vidxs[0]);
    const cv::Vec3f& v1 = vtx( vidxs[1]);
    const cv::Vec3f& v2 = vtx( vidxs[2]);

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
    const double zdist = z.dot(vx0);         // Projected distance off the triangle's surface in direction of normal
    const double zprop = A > 0 ? zdist / sqrt(A) : 0; // sqrt because distance needs to scale linearly (obviously)
    return cv::Vec3f( static_cast<float>(xprop), static_cast<float>(yprop), static_cast<float>(zprop));
}   // end toPropFromAbs


cv::Vec3f ObjModel::toAbsFromProp( int fid, const cv::Vec3f& vp) const
{
    const int* vidxs = fvidxs(fid);
    const cv::Vec3f& v0 = vtx( vidxs[0]);
    const cv::Vec3f& v1 = vtx( vidxs[1]);
    const cv::Vec3f& v2 = vtx( vidxs[2]);
    const double A = calcTriangleArea( v0, v1, v2);
    const cv::Vec3f z = calcFaceNorm(fid);  // Unit length
    return v0 + vp[0]*(v1-v0) + vp[1]*(v2-v1) + static_cast<float>(vp[2]*sqrt(2*A))*z;
}   // end toAbsFromProp


bool ObjModel::isEdgeVertex( int vidx, bool assume2DManifold) const
{
    bool isEdge = false;

    if ( assume2DManifold)
        isEdge = cvtxs(vidx).size() > faces(vidx).size();
    else
    {   // Not assuming a 2D manifold so need to check every edge.
        for ( int cv : cvtxs(vidx))
        {
            if ( nspolys( vidx, cv) == 1)
            {
                isEdge = true;
                break;
            }   // end if
        }   // end for
    }   // end else

    return isEdge;
}   // end isEdgeVertex


bool ObjModel::isManifoldEdge( int v0, int v1) const
{
    if ( !hasEdge(v0,v1))
        return false;
    return spolys( v0, v1).size() != 2;
}   // end isManifoldEdge


bool ObjModel::isManifoldEdge( int eid) const
{
    int v0, v1;
    if ( !edge( eid, v0, v1))
        return false;
    return isManifoldEdge( v0, v1);
}   // end isManifoldEdge


int ObjModel::oppositePoly( int fid, int vi, int vj) const
{
    if ( _faces.count(fid) == 0)
        return -1;
    const IntSet& sfs = spolys( vi, vj);
    if ( sfs.size() != 2) // Assumes exactly two polygons using the specified edge.
        return -1;
    const int bf = *sfs.begin();
    return bf != fid ? bf : *(++sfs.begin());
}   // end oppositePoly


const IntSet& ObjModel::faces( int vi) const
{
    if ( _vtxs.count(vi) == 0)
        return EMPTY_INT_SET;
    return _v2f.at(vi);
}   // end faces


int ObjModel::lookupVertex( const cv::Vec3f& v) const
{
    const size_t key = hash( transform( _imat, v), HASH_NDP);  // Lookup after passing through inverse transform matrix
    return _v2id.count( key) > 0 ? _v2id.at(key) : -1;
}   // end lookupVertex


int ObjModel::lookupVertex( float x, float y, float z) const
{
    return lookupVertex( cv::Vec3f(x,y,z));
}   // end lookupVertex


int ObjModel::lookupUVertex( const cv::Vec3f& v) const
{
    const size_t key = hash( v, HASH_NDP);  // Lookup as is.
    return _v2id.count( key) > 0 ? _v2id.at(key) : -1;
}   // end lookupUVertex


int ObjModel::lookupUVertex( float x, float y, float z) const
{
    return lookupUVertex( cv::Vec3f(x,y,z));
}   // end lookupUVertex


const IntSet& ObjModel::cvtxs( int vid) const
{
    return _v2v.count(vid) == 0 ?  EMPTY_INT_SET : _v2v.at(vid);
}   // end cvtxs



void ObjModel::showDebug( bool withDetails) const
{
    // Print vertex info
    std::cerr << "===============[ RFeatures::ObjModel ]===============" << std::endl;
    const int nv = numVtxs();
    const int nf = numPolys();

    if ( withDetails)
    {
        std::cerr << " Vertices:" << std::endl;
        for ( int vid = 0; vid < nv; ++vid)
        {
            const cv::Vec3f& v = vtx( vid);
            std::cerr << "\tVTX_" << vid << "): x=" << v[0] << ", y=" << v[1] << ", z=" << v[2] << " [VTX connections:";
            // Show connected vertices
            const IntSet& cvs = cvtxs( vid);
            std::for_each( std::begin(cvs), std::end(cvs), [](int cv){ std::cerr << " " << cv;});
            std::cerr << "]" << std::endl;
        }   // end for
        std::cerr << std::endl;
    }   // end if

    const IntSet& matIds = materialIds();
    std::cerr << " Model has " << matIds.size() << " materials" << std::endl;
    for ( int matId : matIds)
    {
        const ObjMaterial& mat = _mats.at(matId);
        std::cerr << " Material has " << mat._uvs.size() << " UV coordinates referencing it" << std::endl;
    }   // end for

    if ( withDetails)
    {
        std::cerr << " Face vertex & UV indices:" << std::endl;
        for ( int fid = 0; fid < nf; ++fid)
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
    std::cerr << " Transform Matrix:" << std::endl;
    std::cerr << " " << _tmat << std::endl;
    std::cerr << " --------------------------------------------------- " << std::endl;
    std::cerr << " # vertices =      " << std::setw(8) << nv << std::endl;
    std::cerr << " _vids.size() =    " << std::setw(8) << _vids.size() << std::endl;
    std::cerr << " _vtxs.size() =    " << std::setw(8) << _vtxs.size() << std::endl;
    std::cerr << " _v2id.size() =    " << std::setw(8) << _v2id.size() << std::endl;
    std::cerr << " _v2v.size() =     " << std::setw(8) << _v2v.size() << std::endl;
    std::cerr << " _v2e.size() =     " << std::setw(8) << _v2e.size() << std::endl;
    std::cerr << " _v2f.size() =     " << std::setw(8) << _v2f.size() << std::endl;
    std::cerr << " _tvtxs.size() =   " << std::setw(8) << _tvtxs.size() << std::endl;
    std::cerr << " --------------------------------------------------- " << std::endl;
    std::cerr << " # polygons =      " << std::setw(8) << nf << std::endl;
    std::cerr << " _fids.size() =    " << std::setw(8) << _fids.size() << std::endl;
    std::cerr << " _faces.size() =   " << std::setw(8) << _faces.size() << std::endl;
    std::cerr << " _f2id.size() =    " << std::setw(8) << _f2id.size() << std::endl;
    std::cerr << " _f2m.size() =     " << std::setw(8) << _f2m.size() << std::endl;
    std::cerr << " --------------------------------------------------- " << std::endl;
    std::cerr << " # edges =         " << std::setw(8) << _edges.size() << std::endl;
    std::cerr << " _eids.size() =    " << std::setw(8) << _eids.size() << std::endl;
    std::cerr << " _edges.size() =   " << std::setw(8) << _edges.size() << std::endl;
    std::cerr << " _e2id.size() =    " << std::setw(8) << _e2id.size() << std::endl;
    std::cerr << " _e2f.size() =     " << std::setw(8) << _e2f.size() << std::endl;
    std::cerr << "=====================================================" << std::endl;
}   // end showDebug
