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

#ifndef RFEATURES_OBJECT_MODEL_H
#define RFEATURES_OBJECT_MODEL_H

#include "rFeatures_Export.h"
#include "VectorFloatKeyHashing.h"

#include <boost/shared_ptr.hpp>
#include <boost/unordered_set.hpp>

#ifdef _WIN32
// Disable warnings about standard template library specialisations not being exported in the DLL interface
#pragma warning( disable : 4251)
#pragma warning( disable : 4275)
#endif

typedef boost::unordered_set<int> IntSet;


namespace RFeatures
{

// A single triangular polygon (face) of the object model
struct rFeatures_EXPORT ObjPoly
{
    ObjPoly();
    ObjPoly( int v0, int v1, int v2);

    bool operator==( const ObjPoly& p) const;                   // Two faces are the same if they share the same vertices.
    bool getOpposite( int v0, int& other0, int& other1) const;  // Get other 2 vertex IDs that aren't v0. Returns false iff not found.
    int getOpposite( int v0, int v1) const;                     // Returns the vertex that isn't v0 or v1 (or -1 if not found).

    // Vertex indices giving the face (stored in ascending order). Do not access this element
    // directly - use ObjModel::getFaceVertices instead since this will always return the
    // vertices in the correct order (i.e. texture mapping order) if the face is texture mapped.
    int fvindices[3];
};  // end struct


// Define an edge as the end point vertex IDs
struct rFeatures_EXPORT Edge
{
    Edge();
    Edge( int u0, int u1);
    bool operator==( const Edge& e) const;
    int v0, v1; // Vertex indices
};  // end struct



class rFeatures_EXPORT ObjModel
{
public:
    /********************************************************************************************************************/
    /****** Instantiation interface *************************************************************************************/

    typedef boost::shared_ptr<ObjModel> Ptr;

    // Create and return a new object model ready for data population.
    // floatPrecision: the spatial precision with which to store point data when supplied.
    static Ptr create( int floatPrecision=6);

    // Create a deep copy of the given model. If the material textures shouldn't be
    // shared (i.e., the texture maps should be cloned), set shareMaterials false.
    static Ptr copy( const ObjModel::Ptr toBeCopied, bool shareMaterials=true);

    Ptr clone( bool shareMaterials=true);   // Member version of copy

    // Returns the floating point precision used to map points in discrete space.
    int getSpatialPrecision() const { return _fltPrc;}


    /********************************************************************************************************************/
    /****** Vertices interface ******************************************************************************************/

    const IntSet& getVertexIds() const { return _vtxIds;}
    size_t getNumVertices() const { return _vtxIds.size();}

    int addVertex( const cv::Vec3f& vertex);        // Add a vertex, returning its index or -1 if vertex values are NaN.
    int addVertex( double x, double y, double z);   // Convenience - values truncated to required float precision!

    // Remove a vertex. Client MUST first remove the associated faces (which removes texture offsets).
    // If function getConnectedVertices returns a non-empty set for the vertex, this function
    // will fail. See functions getFaceIds and unsetFace below.
    bool removeVertex( int vidx);

    // Adjust vertex vidx to be in a new position.
    bool adjustVertex( int vidx, const cv::Vec3f& newPos);
    bool adjustVertex( int vidx, double x, double y, double z);

    const cv::Vec3f& getVertex( int vid) const { return _verts.at(vid);}
    const cv::Vec3f& vtx( int vid) const { return _verts.at(vid);}  // Synonymous with getVertex

    // Return the vertex index for a given position vector.
    // The position of the vertex must be exact. Returns -1 if not found.
    // If wanting the nearest vertex index to v, use ObjModelKDTree::findClosestVertexId(v).
    int lookupVertexIndex( const cv::Vec3f& v) const;

    // Returns the set of vertex indices that are connected to the parameter vertex.
    const IntSet& getConnectedVertices( int vid) const;


    /********************************************************************************************************************/
    /****** Faces interface *********************************************************************************************/

    const IntSet& getFaceIds() const { return _faceIds;}
    size_t getNumFaces() const { return _faceIds.size();}

    // Make a face from already added vertices. Returns index of created face (or index of face already created
    // with those vertices) or -1 if face could not be created because the vertices referenced do not yet exist.
    int setFace( const int* vidxs);
    int addFace( const int* vidxs) { return setFace(vidxs);}
    int setFace( int v0, int v1, int v2);
    int addFace( int v0, int v1, int v2) { return setFace(v0,v1,v2);}

    bool removeFace( int fid);

    const ObjPoly& getFace( int faceId) const;      // Get the specified polygon (assertion checked)
    const ObjPoly& poly( int faceId) const;         // Get the specified polygon (not assertion checked)

    // If face fid is texture mapped, return the texture mapped ordering of the face vertices.
    // Returns face.vindices otherwise and NULL if faceId is invalid.
    const int* getFaceVertices( int faceId) const;

    int getFaceId( int v0, int v1, int v2) const;   // Returns face ID if exists
    const IntSet& getFaceIds( int vid) const;       // Face IDs only for a particular vertex

    // Given a face ID, returns the connectivity metric as the sum of the number of other
    // faces the vertices of this face are connected to. Returns -1 if faceId is invalid.
    int getFaceConnectivityMetric( int faceId) const;

    // Returns the number of faces that the given vertex is used with.
    int getVertexFaceCount( int vid) const;

    // On return, sets fids to the face indices sharing an edge with faceId (does not add
    // faceId itself). Returns the number of adjacent faces added to fids. 
    size_t findAdjacentFaces( int faceId, IntSet& fids) const;

    // Given two vertex indices vx and vy, return the number of faces they share.
    // If this number is zero, then the vertices are not directly connected.
    // If this number is one, then the vertices share an edge on the model's boundary.
    // If this number is greater than two then neither of the vertices are on a "flat"
    // section of the model.
    // Vertex x is defined as flat IFF for all vertices y \in Y where Y is the set of
    // of all vertices directly connected to x, getNumSharedFaces(x,y) <= 2.
    int getNumSharedFaces( int vx, int vy) const;

    // Get the face indices of the faces shared between the two given vertices.
    const IntSet& getSharedFaces( int vi, int vj) const;


    /********************************************************************************************************************/
    /****** Edges interface *********************************************************************************************/

    const IntSet& getEdgeIds() const { return _edgeIds;}
    size_t getNumEdges() const { return _edgeIds.size();}

    const Edge& getEdge( int edgeId) const;
    const IntSet& getEdgeIds( int vid) const;
    bool hasEdge( int vi, int vj) const;
    int getEdgeId( int vi, int vj) const;

    // Connect up vertices vi and vj and return the edge ID or the existing edge ID if already connected.
    // Checks to see if connecting vi and vj makes a triangle; if so, creates one if not already present.
    int setEdge( int vi, int vj);

    // Removes the given edge and any adjacent faces. Does NOT remove vertices!
    bool unsetEdge( int edgeId);


    /********************************************************************************************************************/
    /****** Materials interface *****************************************************************************************/

    // Each Material defines different texture maps: ambient, diffuse, and specular maps.
    const IntSet& getMaterialIds() const { return _materialIds;}
    size_t getNumMaterials() const { return getMaterialIds().size();}

    int addMaterial();                      // Add a new material, returning its index.
    bool removeMaterial( int materialID);   // Returns true iff material was present and was removed.
    bool removeAllMaterials();              // Removes all materials set on this object.

    bool addMaterialAmbient( int materialID, const cv::Mat&);   // Add a ambient texture for given material.
    bool addMaterialDiffuse( int materialID, const cv::Mat&);   // Add a diffuse texture for given material.
    bool addMaterialSpecular( int materialID, const cv::Mat&);  // Add a specular texture for given material.
    const std::vector<cv::Mat>& getMaterialAmbient( int materialID) const;
    const std::vector<cv::Mat>& getMaterialDiffuse( int materialID) const;
    const std::vector<cv::Mat>& getMaterialSpecular( int materialID) const;

    // The faces store vertex IDs in ascending order, but for visualisation the clockwise order of these vertices is needed.
    // Use this function to set the ordering of texture offsets (uvs) that correspond with the given order of vertices (vidxs).
    bool setOrderedFaceUVs( int materialID, int faceId, const int vidxsOrder[3], const cv::Vec2f uvsOrder[3]);
    bool setOrderedFaceUVs( int materialID, int faceId, int v0, const cv::Vec2f& uv0,
                                                        int v1, const cv::Vec2f& uv1,
                                                        int v2, const cv::Vec2f& uv2);

    // Get the material for the given face (not set until setOrderedFaceUVs() called).
    // Returns -1 if no material set for the given face.
    int getFaceMaterialId( int faceId) const;

    // Get the texture UV IDs from the given face or NULL if this face has no UV mappings.
    // Get the specific material these UV IDs relate to with getFaceMaterialId.
    const int* getFaceUVs( int faceId) const;

    // Return a specific UV.
    const cv::Vec2f& uv( int materialID, int uvID) const;

    // Get the set of faces that map material materialID.
    const IntSet& getMaterialFaceIds( int materialID) const;

    // Get all texture UV identifiers from the given material.
    const IntSet& getUVs( int materialID) const;


    /********************************************************************************************************************/
    /****** Utilities/Misc **********************************************************************************************/

    // Find the position within the bounds of poly fid that v is closest to and return it.
    cv::Vec3f projectToPoly( int fid, const cv::Vec3f& v) const;

    // Returns true iff provided point is within the triangle (or its perimeter).
    // Does this by checking the sum of the areas of the three triangles constructed as
    // (v,a,b), (v,a,c), and (v,b,c) where a,b,c are the vertices of the triangle.
    // If area(v,a,b) + area(v,a,c) + area(v,b,c) > area(a,b,c), v must be outside of the triangle.
    bool isVertexInsideFace( int fid, const cv::Vec3f &v) const;

    void showDebug( bool withDetails=false) const;


private:
    const int _fltPrc;      // Vertex floating point storage precision for looking up coincident points.
    int _vCounter;          // Vertex counter.
    int _faceCounter;       // Face counter.
    int _edgeCounter;       // Edge counter.
    int _materialCounter;   // Material counter.

    IntSet _vtxIds;                                 // All vertex IDs
    boost::unordered_map<int, cv::Vec3f> _verts;    // Vertex positions.
    Key3LToIntMap _verticesToUniqIdxs;              // How vertices map to entries in _verts

    struct HashObjPoly : std::unary_function<ObjPoly, size_t> { size_t operator()( const ObjPoly&) const;};
    struct HashEdge    : std::unary_function<Edge,    size_t> { size_t operator()( const Edge&) const;};

    IntSet _faceIds;
    boost::unordered_map<int, ObjPoly> _faces;                  // Faces indexed by vertex index.
    boost::unordered_map<ObjPoly, int, HashObjPoly> _faceMap;   // Faces to IDs for reverse lookup.
    boost::unordered_map<int, IntSet > _vtxToFaces;             // Vertex indices to face indices.
    boost::unordered_map<int, IntSet > _faceEdgeIdxs;           // How face indices map to edge indices (which faces add which edges).

    // The faces connected to each vertex in set Y that are also connected to
    // vertex x where every y in Y is directly connected to x.
    boost::unordered_map<int, boost::unordered_map<int, IntSet> > _vtxConnectionFaces;

    IntSet _edgeIds;
    boost::unordered_map<int, Edge> _edges;             // Edges.
    boost::unordered_map<Edge, int, HashEdge> _edgeMap; // Edges to IDs for reverse lookup.
    boost::unordered_map<int, IntSet > _vtxToEdges;     // vertices to _edges indices.
    boost::unordered_map<int, IntSet > _edgesToFaces;   // Edge IDs to face IDs.
    boost::unordered_map<int, IntSet > _vtxConnections; // The other vertices each vertex is connected to.
    int connectEdge( int v0, int v1);
    void removeEdge( int eidx);
    void removeFaceEdges( int);

    IntSet _materialIds;
    struct Material;
    boost::unordered_map<int, Material*> _materials; // Materials mapped by ID
    boost::unordered_map<int, int> _faceMaterial;   // Map face IDs to material IDs
    void removeFaceUVs( int, int);

    explicit ObjModel( int fltPrc);
    virtual ~ObjModel();
    void setVertexFaceConnections( int, int, int, int);
    void unsetVertexFaceConnections( int, int, int, int);

    ObjModel( const ObjModel&); // No copy
    ObjModel& operator=( const ObjModel&);  // No copy
    class Deleter;
};  // end class


}   // end namespace

#endif
