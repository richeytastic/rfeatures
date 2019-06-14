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

#ifndef RFEATURES_OBJECT_MODEL_H
#define RFEATURES_OBJECT_MODEL_H

#include "ObjModelInternal.h"

namespace RFeatures {

class rFeatures_EXPORT ObjModel
{
public:
    using Ptr = std::shared_ptr<ObjModel>;

    /********************************************************************************************************************/
    /****** Creating / Copying ******************************************************************************************/
    /********************************************************************************************************************/
    // Create and return a new object model ready for data population.
    // floatPrecision: the spatial precision with which to store point data when supplied.
    static Ptr create( int floatPrecision=6);

    // Create and return a deep copy of this model. If the material textures should not
    // be shared (i.e., the texture maps should be cloned), set shareMaterials false.
    Ptr deepCopy( bool shareMaterials=true) const;

    // If this object has non sequential IDs for its vertices, polygons, or edges, then this
    // function returns a deep copy of this object where all IDs are indexed sequentially
    // on [0,N) where N is the corresponding number of vertices/polygons/edges. If this object
    // already has all its IDs sequentially ordered, this function is equivalent to deepCopy.
    Ptr repackedCopy( bool shareMaterials=true) const;

    bool hasSequentialIds() const { return hasSequentialVertexIds() && hasSequentialFaceIds() && hasSequentialEdgeIds();}

    // Returns the floating point precision used to map points in discrete space.
    int spatialPrecision() const { return _fltPrc;}


    /********************************************************************************************************************/
    /****** Vertices ****************************************************************************************************/
    /********************************************************************************************************************/
    inline int numVtxs() const { return int(_vtxs.size());}
    inline const IntSet& vtxIds() const { return _vids;}

    inline const cv::Vec3f& vtx( int vidx) const { return _vtxs.at(vidx);}

    // Some algorithms require sequential vertex IDs so that vertex IDs can be treated as indices.
    inline bool hasSequentialVertexIds() const { return numVtxs() == _vCounter;}

    // Add a vertex and return its ID, or return -1 (and don't add) if any of the position coordinates
    // are NaN. A new ID is assigned and returned for new position vertices. However, if the position
    // matches an existing vertex, then no new vertex is added and the ID of that vertex is returned.
    int addVertex( float x, float y, float z);
    int addVertex( const cv::Vec3f& vertex);

    // Remove a vertex. Client MUST first remove the associated faces (which removes texture offsets).
    // If function cvtxs returns a non-empty set for the vertex, this function will fail. Note that
    // removing a vertex also removes its ID from this object which can affect whether algorithms
    // that require this object to have sequential vertex IDs work.
    bool removeVertex( int vidx);

    // Adjust vertex vidx to be in a new position.
    bool adjustVertex( int vidx, const cv::Vec3f& newPos);
    bool adjustVertex( int vidx, float x, float y, float z);

    // Multiply the component of each vertex by f.
    bool scaleVertex( int vidx, float f);

    // Return the ID for given vertex v (position must be exact).
    // Returns -1 if not found. In general, this function should not be used
    // and ObjModelKDTree::findClosestVertexId(v) should be preferred to return
    // the ID of the vertex closest to the given vertex.
    int lookupVertex( const cv::Vec3f&) const;

    // Returns the set of vertex indices that are connected to the parameter vertex.
    inline const IntSet& cvtxs( int vid) const { return _v2v.at(vid);}


    /********************************************************************************************************************/
    /****** Polys *******************************************************************************************************/
    /********************************************************************************************************************/
    inline int numPolys() const { return int(_faces.size());}
    inline const IntSet& faces() const { return _fids;}

    inline bool hasSequentialFaceIds() const { return numPolys() == _fCounter;}

    // Make a poly from already added vertices. Returns ID of created poly (or ID of poly already created
    // with those vertices) or -1 if poly could not be created because referenced vertices don't yet exist.
    // The order of vertices on a face matters since this defines its normal vector.
    int addFace( const int* vidxs);
    int addFace( int v0, int v1, int v2);

    // Remove a poly (also removing its ID).
    bool removePoly( int);

    // Get the specific poly.
    inline const ObjPoly& face( int id) const { return _faces.at(id);}

    // Convenience function to return given poly's vertices in stored order or null if ID invalid.
    const int* fvidxs( int id) const;

    // Returns poly ID if exists with given vertices or -1 if not.
    int face( int v0, int v1, int v2) const;

    // Reverse lookup a poly ID from a poly or -1 if not present.
    inline int rface( const ObjPoly& fc) const { return _f2id.count(fc) > 0 ? _f2id.at(fc) : -1;}

    // Returns the IDs of the polys that use the given vertex.
    const IntSet& faces( int vidx) const;

    // Reverse the order of the vertices set on the given poly. Will have the effect of flipping
    // the direction of the normal returned by calcFaceNorm. Ensures that texture mapping also
    // remains correctly associated (matched to the poly's vertices).
    void reversePolyVertices( int id);

    // Given the ordering of vertices on the face, calculate and return the unit normal.
    cv::Vec3f calcFaceNorm( int fid) const;

    // As calcFaceNorm, but place the i and j unit vectors used in the calculation of the returned vector
    // in the out parameters. Note that unit vectors i and j are simply normalized difference vectors of
    // the polygon edges and so are almost certainly not orthogonal to one another! If a set of orthonormal
    // vectors are needed, the client should take the cross product of either vi OR vj with the returned
    // vector since the returned norm is orthogonal to both vi and vj.
    cv::Vec3f calcFaceNorm( int fid, cv::Vec3f& vi, cv::Vec3f& vj) const;

    // Calculate and return the area of the given face.
    double calcFaceArea( int fid) const;

    // Given two vertex indices vx and vy, return the number of faces they share.
    // If this number is zero, then the vertices are not directly connected.
    // If this number is one, then the vertices share an edge on the model's boundary.
    // If this number is greater than two then neither of the vertices are on a "flat"
    // section of the model.
    // Vertex x is defined as flat IFF for all vertices y \in Y where Y is the set of
    // of all vertices directly connected to x, nspolys(x,y) <= 2.
    int nspolys( int vx, int vy) const;
    int nspolys( int edgeId) const;

    // Get the IDs of the polygons shared between the given vertices.
    const IntSet& spolys( int vi, int vj) const;
    const IntSet& spolys( int edgeId) const;

    // If edge i,j (vertex IDs) shares exactly two triangles (T0 = i,j,k and T1=i,j,l),
    // this function flips the edge between the two triangles to be k,l instead of i,j.
    // This changes the shape of the surface between the triangles, and effectively creates
    // a replacement pair of triangles using the same four vertices but with different
    // vertex membership. This function does NOT add or remove face IDs so existing
    // references to the set of face IDs from function faces() will not be corrupted.
    // NB: Concerning material mappings, if the two faces are mapped to the same material,
    // the texture coordinates will be updated to reflect the change in geometry of the
    // two triangles. HOWEVER, if the triangles are mapped to different materials, the
    // material mappings on the newly adjusted triangles will be removed!
    // Returns true iff two triangles are successfully flipped in this manner.
    // Returns false in all other cases (and the model is unchanged).
    bool flipFacePair( int vi, int vj);


    /********************************************************************************************************************/
    /****** Edges *******************************************************************************************************/
    /********************************************************************************************************************/
    inline int numEdges() const { return int(_edges.size());}
    inline const IntSet& edgeIds() const { return _eids;}

    inline bool hasSequentialEdgeIds() const { return numEdges() == _eCounter;}

    bool hasEdge( int vi, int vj) const;
    inline const ObjEdge& edge( int edgeId) const { return _edges.at(edgeId);}
    inline const IntSet& edgeIds( int vid) const { return _v2e.at(vid);}
    bool edge( int edgeId, int& v0, int& v1) const;
    int edgeId( int vi, int vj) const;   // Returns -1 if edge doesn't exist.

    // Connect up vertices vi and vj and return the edge ID or the existing edge ID if already connected.
    // Checks to see if connecting vi and vj makes a triangle; if so, creates one if not already present.
    // NB the order of vi and vj is important if the caller cares about consistent polygon normals!
    // When setting this edge causes one or more new polygons to be created, the polygons are set with
    // vertex ordering vs,vi,vj where vs \in VS and VS is the set of vertices already connected by an
    // edge to both vi and vj.
    int addEdge( int vi, int vj);

    // Removes the given edge and any adjacent faces. Does NOT remove vertices!
    // Returns true if the edge existed and was removed.
    bool removeEdge( int edgeId);
    bool removeEdge( int vi, int vj);


    /********************************************************************************************************************/
    /****** Materials ***************************************************************************************************/
    /********************************************************************************************************************/
    const IntSet& materialIds() const { return _mids;}

    // Note that material IDs are not necessarily in sequential order - even for a repacked obj.
    inline size_t numMats() const { return _mids.size();}

    // Set texture for a new material (resized so that they're no wider/higher than maxDim cols/rows)
    // and return its index. Returns -1 and no material is added if the given texture is empty.
    int addMaterial( const cv::Mat&, size_t maxDim=4096);
    const cv::Mat& texture( int mid) const;

    void removeMaterial( int mid);   // Removes the material with given id.
    void removeAllMaterials();       // Removes all materials set on this object.

    // Copy in the materials from the parameter ObjModel to this one.
    void copyInMaterials( const ObjModel*, bool shareMaterials=true);

    // Merge the materials on this object into a single material. Returns the number of materials that
    // were merged (== numMats() prior to calling). This creates a single large texture image from
    // the individual texture across the materials. This also changes face material membership.
    // This function is available because many rendering schemes have problems when it comes to mapping
    // more than one texture to an object. In particular, multi-texturing support in VTK (as of version 7.1)
    // is unreliable, and rendering of 3D objects in PDFs (as embedded U3D scenes) can result in unwelcome
    // lighting issues. This function circumvents such issues by mapping just a single texture.
    size_t mergeMaterials();

    // Set the ordering of texture offsets (uvs) that correspond with the order of vertices specified in addFace.
    bool setOrderedFaceUVs( int mid, int fid, const cv::Vec2f uvs[3]);
    bool setOrderedFaceUVs( int mid, int fid, const cv::Vec2f& uv0, const cv::Vec2f& uv1, const cv::Vec2f& uv2);

    // Get the material for the given face (not set until setOrderedFaceUVs() called).
    // Returns -1 if no material set for the given face.
    int faceMaterialId( int fid) const;

    // For assumed geometry edge v0-->v1, returns the number of associated texture edges from all of the materials mapped to
    // this edge. This primarily depends upon the number of materials mapped to the adjacent faces, and the number of faces
    // shared by this edge, but it also depends upon the nature of the material/texture mapping itself. It is possible that
    // a single geometric (3D) edge shared between two (or more) faces - all of which are mapped to the same material - can
    // be mapped to two (or more) texture (2D) edges. Such edges can present difficulties with remeshing operations
    // (e.g. flipFacePair) because modifying the edge in 3D necessitates a more complicated modification of the "texture"
    // edges so that the texture mapping remains visually consistent with the underlying geometry. This function can be
    // used to test for this situation and so avoid carrying out geometric modifications on these kind of edges.
    size_t numTextureEdges( int v0, int v1) const;

    // Get the texture UV IDs from the given face or null if this face has no UV mappings.
    // The specific material these UV IDs relate to is found with faceMaterialId( fid).
    const int* faceUVs( int fid) const;

    // Return a specific UV.
    const cv::Vec2f& uv( int materialID, int uvID) const;

    // Return a specific UV from a face vertex i on [0,2]. Only valid if the face has a material!
    const cv::Vec2f& faceUV( int fid, int i) const;

    // Get the set of faces that map just to the given material.
    const IntSet& materialFaceIds( int mid) const;

    // Get all texture UV identifiers from the given material.
    const IntSet& uvs( int mid) const;

    // For a face with an existing texture, return the texture coord for the vertex in the plane of that face
    // (vertex doesn't need to be inside the face). Returns cv::Vec2f(-1,-1) if fid has no texture coords.
    cv::Vec2f calcTextureCoords( int fid, const cv::Vec3f& v) const;

    /********************************************************************************************************************/
    /****** Utilities / Misc ********************************************************************************************/
    /********************************************************************************************************************/
    // Find the position within the bounds of poly fid that p is closest to and return it.
    cv::Vec3d projectToPoly( int fid, const cv::Vec3d& p) const;

    // For function toPropFromAbs, given a point relative to a polygon fid, return three scalars a, b, and c
    // giving the proportions along edges v0-->v1, v1-->v2, and height off the polygon as a proportion of the
    // square root of twice the triangle's area respectively. Position p can then be restored later relative
    // to ANY polygon using toAbsFromProp as:
    // p = v0 + a*(v1-v0) + b*(v2-v1) + c*sqrt(2A)*w,
    // where A is the area of the restoring triangle polygon, and w is the unit vector normal to it.
    // This works whether or not point p is inside the given polygon.
    cv::Vec3d toPropFromAbs( int fid, const cv::Vec3d&) const;
    cv::Vec3d toAbsFromProp( int fid, const cv::Vec3d&) const;

    // Returns true iff provided point is within the triangle (or its perimeter).
    // Does this by checking the sum of the areas of the three triangles constructed as
    // (v,a,b), (v,a,c), and (v,b,c) where a,b,c are the vertices of the triangle.
    // If area(v,a,b) + area(v,a,c) + area(v,b,c) > area(a,b,c), v must be outside of the triangle.
    // Only works if v is in the plane of polygon fid!
    bool isVertexInsideFace( int fid, const cv::Vec3d&) const;

    // Returns true iff vidx is paired with some other vertex forming an edge used just by a single polygon.
    // By default, this function checks other vertices connected to vidx until it finds an edge pair used
    // only by a single triangle making the function linear in the number of connected vertices.
    // However, if assume2DManifold is set to true, it is taken that the vertex belongs to a local 2D manifold
    // (i.e. its set of adjacent polygons are topologically homeomorphic to a 2D plane) which allows a constant
    // time test for edge membership to be employed which simply entails checking if the number of
    // connected vertices is greater than the number of adjacent polygons.
    bool isEdgeVertex( int vidx, bool assume2DManifold=false) const;

    // Returns true iff the edge v0-->v1 exists, and it is shared by 1 or 3 or more triangles.
    bool isManifoldEdge( int v0, int v1) const;
    bool isManifoldEdge( int edgeId) const;

    // Given edge v0-->v1 on face fid, return the other shared face on edge v0-->v1 that isn't fid.
    // If the edge does not share exactly one other face, return -1.
    int oppositePoly( int fid, int v0, int v1) const;

    // Show info about this model.
    void showDebug( bool showDetail=false) const;

private:
    int _fltPrc;        // Vertex floating point storage precision for looking up coincident points.
    int _vCounter;      // Vertex counter
    int _fCounter;      // Face counter
    int _eCounter;      // Edge counter
    int _mCounter;      // Material counter

    IntSet _vids;                                   // Vertex IDs
    std::unordered_map<int, cv::Vec3f> _vtxs;       // Vertex positions
    Key3LToIntMap _v2id;                            // Reverse lookup vertex IDs

    IntSet _fids;                                   // Face IDs
    std::unordered_map<int, ObjPoly> _faces;        // Faces
    std::unordered_map<ObjPoly, int, HashObjPoly> _f2id;  // Reverse lookup face IDs

    IntSet _eids;                                   // Edge IDs
    std::unordered_map<int, ObjEdge> _edges;        // Edges
    std::unordered_map<ObjEdge, int, HashObjEdge> _e2id;   // Reverse lookup edge IDs

    std::unordered_map<int, IntSet> _v2v;           // Vertex to vertex connections (symmetric)
    std::unordered_map<int, IntSet> _v2e;           // Vertex ID to edge ID lookup
    std::unordered_map<int, IntSet> _v2f;           // Vertex ID to face ID lookup
    std::unordered_map<int, IntSet> _e2f;           // Edge ID to face ID lookup

    IntSet _mids;                                   // Material IDs
    std::unordered_map<int, ObjMaterial> _mats;     // Materials mapped by ID
    std::unordered_map<int, int> _f2m;              // Face IDs to material IDs

    int _connectEdge( int, int);
    void _connectEdge( int, int, int);
    void _removeEdge( int);
    void _removeFaceUVs( int, int);
    void _addMaterial( int, const cv::Mat&, size_t);

    explicit ObjModel( int fltPrc);
    ObjModel( const ObjModel&) = default;
    ObjModel& operator=( const ObjModel&) = default;
    virtual ~ObjModel();
};  // end class

}   // end namespace

#endif
