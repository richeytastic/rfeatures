#ifndef RFEATURES_OBJ_MODEL_KD_TREE_H
#define RFEATURES_OBJ_MODEL_KD_TREE_H

#include "ObjModel.h"

namespace RFeatures
{

class rFeatures_EXPORT ObjModelKDTree
{
public:
    typedef boost::shared_ptr<ObjModelKDTree> Ptr;

    // Don't modify the given model while this data structure in use.
    static Ptr create( const ObjModel::Ptr);
    static Ptr create( const ObjModel::Ptr, const IntSet& vidxs);   // Create from just the given vertex IDs

    const ObjModel::Ptr getObject() const { return _model;}

    // Find the closest vertex ID on the model that are closest to p.
    // If not NULL, set sqdis on return to be the squared distance to the vertex.
    int find( const cv::Vec3f& p, float *sqdis=NULL) const;
    int find( const cv::Vec3d& p, float *sqdis=NULL) const;

    // Find the closest n vertex IDs on the model that are closest to p. n is specified by
    // preallocating nvidxs to the desired size. If sqdis is given, it must also have
    // the same size as nvidxs. Returns the actual number of points found which may be
    // less than n.
    int findn( const cv::Vec3f& p, std::vector<int>& nvidxs, std::vector<float>* sqdis=NULL) const;
    int findn( const cv::Vec3d& p, std::vector<int>& nvidxs, std::vector<float>* sqdis=NULL) const;

private:
    const ObjModel::Ptr _model;
    class Impl; // pimple idiom (who doesn't love saying that...)
    Impl *_impl;

    explicit ObjModelKDTree( const ObjModel::Ptr);
    ObjModelKDTree( const ObjModel::Ptr, const IntSet&);
    ~ObjModelKDTree();
    ObjModelKDTree( const ObjModelKDTree&); // No copy
    ObjModelKDTree& operator=( const ObjModelKDTree&);  // No copy
    class Deleter;
};  // end class

}   // end namespace

#endif
