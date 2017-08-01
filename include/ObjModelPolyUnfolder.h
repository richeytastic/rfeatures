#ifndef RFEATURES_OBJ_MODEL_POLY_UNFOLDER_H
#define RFEATURES_OBJ_MODEL_POLY_UNFOLDER_H

#include "ObjModel.h"

namespace RFeatures
{

class rFeatures_EXPORT ObjModelPolyUnfolder
{
public:
    // Sets the triangle that represents the unfolding plane using the given
	// model. All of this triangle's vertices will be set in the unfolded set,
    // and so unfoldAlongEdge can be called with any pair of vertices from this
	// triangle and subsequent triangle T in unfoldAlongEdge.
    ObjModelPolyUnfolder( const ObjModel::Ptr, int T);

    const ObjModel::Ptr getObject() const { return _model;}

    // Unfold triangle T along vertex edge v0,v1, causing vertex v2 to
    // be moved into the unfolding plane. Vertices v0 and v1 must already
    // by in the set of unfolded vertices.
    int unfoldAlongEdge( int T, int v0, int v1);

    const cv::Vec3d& getUnfoldedVertex( int vidx) const { return _unfoldedUVs.at(vidx);}
    const cv::Vec3d& getUnfoldingPlane() const { return _planeNormal;}

private:
    const ObjModel::Ptr _model;

    boost::unordered_map<int, cv::Vec3d> _unfoldedUVs;  // Positions of "unfolded" vertices
    cv::Vec3d _planeNormal;    // Normal definining the "unfolding" plane
};  // end class

}   // end namespace

#endif
