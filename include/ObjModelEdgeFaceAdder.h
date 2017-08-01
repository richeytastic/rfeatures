#ifndef RFEATURES_OBJ_MODEL_EDGE_FACE_ADDER_H
#define RFEATURES_OBJ_MODEL_EDGE_FACE_ADDER_H

#include "ObjModel.h"

namespace RFeatures
{

class ObjModelEdgeFaceAdder
{
public:
    explicit ObjModelEdgeFaceAdder( ObjModel::Ptr mod) : _mod(mod) {}

    // The vertex IDs in xyset must already be present in the model.
    void addFaces( const boost::unordered_map<int,IntSet>& xyset);

private:
    ObjModel::Ptr _mod;
    boost::unordered_map<int, boost::unordered_map<int, int> > _edgeUse; // Count number of times each edge is used

    bool setFace( int x, int y, int z);
    bool areSharedFacesJoined( int x, int y, int z) const;
    bool sharesCommonEdge( int f0, int f1) const;
    void init( int x, int y);
    void addTriangle( int x, int y, int z);
};  // end class

}   // end namespace

#endif
