/**
 * A generic object model with texture.
 * Richard Palmer
 * August 2014
 */

#pragma once
#ifndef RFEATURES_OBJECT_MODEL_H
#define RFEATURES_OBJECT_MODEL_H

#include "rFeatures_Export.h"
#include "VectorFloatKeyHashing.h"

#include <boost/shared_ptr.hpp>
#include <boost/unordered_set.hpp>
#ifndef Q_MOC_RUN
#include <boost/foreach.hpp>
#endif

typedef boost::unordered_set<int> IntSet;


namespace RFeatures
{

class ObjModelCleaner;

// A single triangular polygon (face) of the object model
struct rFeatures_EXPORT ObjPoly
{
    static const int NVTX = 3; // Number of vertices (always 3 for a triangular polygon)
    int vindices[NVTX];    // Vertex indices giving the face (should be stored clockwise)
    cv::Vec3f nrmVec;   // Face normal

    // Two faces are the same if they share the same vertices (in whatever order)
    bool operator==( const ObjPoly& p) const;
    size_t calcHash() const;
};  // end struct


// Define an edge as the end point unique vertex IDs
struct rFeatures_EXPORT Edge
{
    Edge();
    Edge( int u0, int u1);

    bool operator==( const Edge& e) const;

    int v0; // Unique vertex index into _uniqVerts
    int v1; // Unique vertex index into _uniqVerts
};  // end struct


class rFeatures_EXPORT ObjModel
{
public:
    typedef boost::shared_ptr<ObjModel> Ptr;

    // Create and return a new object model ready for data population.
    // floatPrecision: the spatial precision with which to store point data when supplied.
    static Ptr create( int floatPrecision=6);

    void setTexture( const cv::Mat texture) { _texture = texture;} // Set this object's texture.
    bool isTextured() const { return !_texture.empty();}    // Check if object has texture

    // Add a vertex, and its (optional) texture offset, returning its index or -1 if unable to add.
    int addVertex( const cv::Vec3f& vertex, const cv::Vec2f& uvTextureOffset=cv::Vec2f(0,0));

    // Remove a vertex and its associated texture offsets. Use MUST first remove the associated faces!
    // If function getConnectedUniqueVertices returns a non-empty set for the vertex, these functions
    // will fail. See functions getFaceIds, getFaceIdsFromUniqueVertex, and unsetFace below.
    bool removeVertex( int vertexId);
    bool removeUniqueVertex( int uvidx);

    const IntSet& getVertexIds() const { return _vtxIds;}
    const IntSet& getUniqueVertexIds() const { return _uvtxIds;}
    const IntSet& getFaceIds() const { return _faceIdxs;}   // Get all face indices
    const IntSet& getEdgeIds() const { return _edgeIdxs;}

    const cv::Vec3f& getVertex( int vid) const { return _uniqVerts.at(_vtxToUvtx.at(vid));}
    const cv::Vec2f& getTextureOffset( int vid) const { return _vtxToTextures.at(vid);}
    const cv::Vec3f& getUniqueVertex( int uvid) const { return _uniqVerts.at(uvid);}
    int getUniqueVertexIdFromNonUnique( int vid) const { return _vtxToUvtx.at(vid);}

    // A vertex is lonely if it is not connected to any faces (i.e. it has no vertex connections).
    // Should only ever be the case prior to setting all faces!
    bool isLonely( int uvid) const;

    // A vertex x is non-flat (i.e. part of a local 3D topology) if there is at least one
    // of its directly connected vertices y where x and y share more than two faces.
    // Therefore, a flat vertex x is one where none of its connected vertices y share
    // with it more than two faces.
    bool isFlat( int uvid) const;

    // Returns true if at least one of the connected (unique) vertices
    // to uvid shares vertex uvid with only a single triangle. Note that
    // a vertex can be a "boundary" vertex and also be non-flat (part of a local 3D topology).
    bool isBoundary( int uvid) const;

    // A flat boundary is one where the boundary condition is fulfilled by exactly
    // two of its directly connected vertices. That is, there exist two members of the set
    // of directly connected vertices to vertex x which share two single faces
    // (or one single face) with vertex x. If the same single face is shared by
    // both of the connected vertices, then vertex x must be a "tip" (see below).
    // A flat boundary vertex has only a single edge passing through it.
    bool isFlatBoundary( int uvid) const;

    // A junction is a kind of boundary vertex that is not 1D and is not flat because it is a point
    // on more than one edge. I.e., there exists more than one way to pass through the vertex, or
    // alternatively, that the vertex is part of more than two separate edges.
    bool isJunction( int uvid) const;

    // A unique vertex is a "tip" if it is used in only a single face. A "tip" is also a flat boundary vertex.
    // Vertex x is a tip if getUniqueVertexFaceCount(x) == 1 (which is simply what this function does).
    bool isTip( int uvid) const;

    // Returns the number of faces that the given unique vertex is used with.
    int getUniqueVertexFaceCount( int uvid) const;

    // Returns the normal at a vertex from the combined normals of the polys it's connected to.
    cv::Vec3f getNormal( int uvid) const;

    /**
     * Make a face from already added vertices.
     * Returns the index of the created face, or -1 signifying that the face could not be created
     * because the vertices referenced do not yet exist OR a face with these vertices is already set!
     * When a face is set, its normal is also created according to the order of the vertices given
     * which should be given clockwise so that a face normal that points outwards from the model is
     * calculated. vidxs: an array of three indices of the vertices giving the three corners of the poly.
     */
    int setFace( const int* vidxs);
    int setFace( int v0, int v1, int v2);
    bool unsetFace( int faceId);    // Unset the face - returns true iff removed.

    const ObjPoly& getFace( int faceId) const;
    const ObjPoly& getUniqueVertexFace( int faceId) const;  // Same as above, but stored face uses unique vertices.

    // Given a face ID, returns the connectivity metric as the sum of the number of other
    // faces the vertices of this face are connected to. Returns -1 if faceId is invalid.
    int getFaceConnectivityMetric( int faceId) const;

    const IntSet& getFaceIds( int vertexId) const;  // Face IDs only for a particular vertex
    const IntSet& getFaceIdsFromUniqueVertex( int uniqueVertexId) const;    // All faces sharing this point in space

    // Returns the set of vertex IDs that are mapped to the given position in space.
    // This set can be larger than a single ID because many points may have the same
    // position (especially in the case of textured objects).
    const IntSet& lookupVertexIndices( const cv::Vec3f& v) const;

    // Return the unique vertex index for a given position vector. Returns -1 if not found.
    int lookupUniqueVertexIndex( const cv::Vec3f& v) const;

    // Given a unique vertex index, lookup all of the vertex/texture indices that use this point.
    const IntSet& lookupTextureIndices( int uniqueVertexId) const;

    size_t getNumVertices() const { return _vtxIds.size();}
    size_t getNumUniqueVertices() const { return _uvtxIds.size();}
    size_t getNumFaces() const { return _faceIdxs.size();}
    size_t getNumEdges() const { return _edgeIdxs.size();}

    const Edge& getEdge( int edgeId) const;

    // Returns the set of unique vertex indices that are connected to the parameter unique vertex.
    const IntSet& getConnectedUniqueVertices( int uvid) const;

    // Given two unique vertex indices uvx and uvy, return the number of faces they share.
    // If this number is zero, then the vertices are not directly connected.
    // If this number is one, then the vertices share an edge on the model's boundary.
    // If this number is greater than two then neither of the vertices are on a "flat"
    // section of the model.
    // Unique vertex x is defined as flat IFF for all vertices y \in Y where Y is the set of
    // of all vertices directly connected to x, getNumSharedFaces(x,y) <= 2.
    int getNumSharedFaces( int uvx, int uvy) const;

    // Get the face indices of the faces shared between the two given unique vertices.
    const IntSet& getSharedFaces( int uvx, int uvy) const;

    // Get this object's texture map (will be empty if not set).
    const cv::Mat& getTexture() const { return _texture;}

    // Create a subset of this model from the given std::vector of unique vertices
    // (duplicate vertices in uvids are ignored).
    // If any of the ids are invalid, a NULL pointer is returned.
    // Textured faces are set with polygons (triangles) having all three of their
    // vertices present (although, all given vertices are added to the returned model).
    Ptr createFromUniqueVertices( const std::vector<int>& uvids) const;

    // Face curvature calculated as the determinant (scalar triple product) of the normals for
    // the three vertices of each face. Curvature on a vertex calculated as the average curvature
    // of its associated faces. Functions aren't const because curvature info is generated
    // and stored as needed as these functions are called for particular faces/vertices.
    double getFaceCurvature( int faceIdx);
    double getUniqueVertexCurvature( int uvidx);

    ~ObjModel();

private:
    const float _fltPrc;    // Vertex floating point storage precision for looking up coincident points
    int _uvCounter;         // Unique vertex counter
    int _vCounter;          // Vertex counter
    int _faceCounter;       // Face counter
    int _edgeCounter;       // Edge counter

    IntSet _vtxIds;     // All vertex IDs
    IntSet _uvtxIds;    // All unique vertex IDs
    boost::unordered_map<int, int> _vtxToUvtx;  // Vertex ID to unique vertex ID mappings (used to be _vertices)
    boost::unordered_map<int, cv::Vec2f> _vtxToTextures;    // Vertex IDs to texture mappings
    boost::unordered_map<int, cv::Vec3f> _uniqVerts;        // Unique vertices positions
    boost::unordered_map<int, double> _uvtxCurvature;       // Unique vertex curvature (generated later)
    boost::unordered_map<int, IntSet > _uvtxToVtx;          // unique vertices to duplicated vtx/tx indices
    boost::unordered_map<int, int> _uvfcounts;        // Unique vertex face counts (number of faces a unique vertex is used in)

    RFeatures::Vec3iToIntMap _verticesToIdxs;   // How vertices map to entries in _uniqVerts

    IntSet _faceIdxs; // _faces.size() == _ufaces.size() == _faceIdxs.size()
    boost::unordered_map<int, const ObjPoly*> _faces;
    boost::unordered_map<int, const ObjPoly*> _ufaces;  // Faces with references to unique vertices (no good for texturing!)
    boost::unordered_map<size_t, std::list<const ObjPoly*> > _ufaceHashes;  // When adding faces, ensure same poly not added twice!
    boost::unordered_map<int, double> _faceCurvature;   // Face curvature (may have no entries until generateCurvature called)
    boost::unordered_map<int, IntSet > _uvtxToFaces;    // Unique vertices to face indices (_faces and _ufaces)
    boost::unordered_map<int, IntSet > _faceEdgeIdxs;   // How face indices map to edge indices (which faces add which edges)
    void removeFaceEdges( int faceIdx);

    IntSet _edgeIdxs;
    boost::unordered_map<int, const Edge*> _uedges;     // Unique edges
    boost::unordered_map<int, IntSet > _uvtxToEdges;    // unique vertices to _uedges indices
    boost::unordered_map<int, IntSet > _edgesToFaces;   // Edge IDs to face IDs
    void setEdge( int faceIdx, int uvidx0, int uvidx1);

    cv::Mat _texture;   // Model texture

    // The other unique vertices each unique vertex is connected to.
    boost::unordered_map<int, IntSet > _uvtxConnections;
    // The faces connected to each vertex in set Y that are also connected to
    // vertex x where every y in Y is directly connected to x.
    boost::unordered_map<int, boost::unordered_map<int, IntSet> > _uvtxConnectionFaces;

    explicit ObjModel( int fltPrc);
    void addFaceFromModel( const ObjModel* model, const ObjPoly& face);
    double calcDeterminant( const ObjPoly&) const;
    void generateFaceCurvature( int fid);
    void generateUniqueVertexCurvature( int uvid);

    friend class ObjModelCleaner;   // Needed for regenerating curvatures
};  // end class

}   // end namespace

#endif
