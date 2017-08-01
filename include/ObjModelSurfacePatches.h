#ifndef RFEATURES_OBJ_MODEL_SURFACE_PATCHES_H
#define RFEATURES_OBJ_MODEL_SURFACE_PATCHES_H

#include "ObjModelKDTree.h"

namespace RFeatures
{

class rFeatures_EXPORT ObjModelSurfacePatches
{
public:
    // Get points from patches around given locations within radius R.
    // Note that the model referenced by the kdtree must have connections between points!
    ObjModelSurfacePatches( const ObjModelKDTree::Ptr, float R);

    const ObjModelKDTree::Ptr getKDTree() const { return _dtree;}

    // Set pset with the M vertex IDs within R from v (if M < 0, get all points).
    // There may be fewer than M points within R of V, and so the actual number of vertex IDs
    // added to pset is returned. (pset is added to without first being cleared).
    // In the case that M > 0, a priority queue is used to ensure that the points
    // closest to v are found. Otherwise, a more efficient algorithm is used to
    // obtain all points within R of v.
    int getPatchVertexIds( const cv::Vec3f& v, IntSet& pset, int M=-1) const;

private:
    const ObjModelKDTree::Ptr _dtree;
    const float _sqR;
};  // end class

}   // end namespace

#endif
