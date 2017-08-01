#ifndef RFEATURES_OBJ_MODEL_SHORTEST_PATH_FINDER_H
#define RFEATURES_OBJ_MODEL_SHORTEST_PATH_FINDER_H

#include "ObjModelKDTree.h"

namespace RFeatures
{

class rFeatures_EXPORT ObjModelShortestPathFinder
{
public:
    explicit ObjModelShortestPathFinder( const ObjModelKDTree::Ptr);

    const ObjModel::Ptr& getKDTree() const { return _kdtree;}

    // Find the shortest path on the model's surface from v0 to v1, placing the output points in pts.
    // Returns the number of points added to pts. Does not clear pts before use!
    int operator()( const cv::Vec3f& v0, const cv::Vec3f& v1, std::vector<cv::Vec3f>& pts) const;

    // As above but endpoints defined to be at the specified vertices.
    int operator()( int v0, int v1, std::vector<cv::Vec3f>& pts) const;

private:
    const ObjModelKDTree::Ptr _kdtree;
};  // end class

}   // end namespace

#endif
