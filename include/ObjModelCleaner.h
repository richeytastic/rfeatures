#ifndef RFEATURES_OBJ_MODEL_CLEANER_H__
#define RFEATURES_OBJ_MODEL_CLEANER_H__

#include "ObjModel.h"
#include "ObjModelTools.h"
#include <boost/unordered_map.hpp>


namespace RFeatures
{

class rFeatures_EXPORT ObjModelCleaner
{
public:
    explicit ObjModelCleaner( ObjModel::Ptr originalToBeCleaned);
    virtual ~ObjModelCleaner();

    ObjModel::Ptr getObjModel() const { return _model;}

    size_t getNumLonelyVertices() const { return _lonely->size();}
    size_t getNumFlatVertices() const { return _flat->size();}
    size_t getNumNonFlatVertices() const { return _nonflat->size();}
    size_t getNumEdgeVertices() const { return _edge->size();}
    size_t getNumCompVertices() const { return _comp->size();}

    // Find and remove faces that cause local 3D topology to arise, and
    // vertices that are unnecessary because all of their connected vertices have 3D topology.
    int remove3D();

    // Find and remove vertices and faces connected to those vertices where
    // the vertex represents a point in space - not a part of a higher dimensional surface.
    int remove1D();

    // All vertices (and associated faces) from the model are removed
    // where they connect to fewer than minVtxFaceConns faces. Returns # of removed vertices.
    int pruneVertices( int minVtxFaceConns=3);

    // Remove vertices and adjoining faces where the vertex has edges
    // greater than u + 0.6s where u is the mean edge length and s is
    // the standard deviation.
    //void todo();  // Maybe

    // Find vertices where the gradient is greater than maxGrad and remove them.
    void removeSpikes( double maxGrad);

    // All vertices (and associated edges) are removed having an absolute
    // gradient at the vertex more than the specified amount. The hole is
    // filled using a replacement vertex that interpolates between the
    // old connected vertices.
    void adjustHighGradientVertices( double maxAbsGradient);

    // Given the provided unique vertices, remove them (and their attached faces)
    // from the model. Returns the modified model.
    void removeUniqueVertices( const IntSet& uvidxs);

    // Clean while copying an original.
    //static ObjModel::Ptr createCleanedCopy( const ObjModel::Ptr original, double maxAbsGrad, int minVtxFaceConns=3);

private:
    ObjModel::Ptr _model;
    IntSet *_lonely, *_flat, *_nonflat, *_edge, *_comp;    // Sets for the different topologies
    void updateUniqueVertexTopology( int uvidx);
    void regatherTopology();
    bool removeVertex( int uvidx);  // Returns true if uvidx was present for removal
    void removeFace( int fid);
    void removeVertexAndFaces( int uvidx);
    bool is3DExtrusion( int uvidx) const;
    void remove1DSet( const IntSet*);

    // A vertex x is extruded if for its set of connected vertices C_x, every edge formed by every
    // member of C_x and x is shared by at least three polygons. In this case, vertex x can be safely removed.
    int removeFlatExtrusions(); // Returns number of extrusions removed

    int removeNonFlatMakingEdges( int uvidx);
    int removeJunctionConnections( int uvidx); // Returns number of connected vertices removed
    int removeSurfaceJoin( int uvidx);
};  // end class

}   // end namespace

#endif
