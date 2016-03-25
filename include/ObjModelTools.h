#pragma once
#ifndef RFEATURES_OBJ_MODEL_TOOLS_H
#define RFEATURES_OBJ_MODEL_TOOLS_H

#include <vector>
#include <algorithm>
#include <boost/foreach.hpp>
#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>
#include "ObjModel.h"
#include "rFeatures_Export.h"

namespace RFeatures
{

// Return that portion of the model having points connected to others within the given boundary,
// where inUvtxId is a starting unique index to grow the surface.
// The topology of the model must be homologous to a 2D plane - i.e. if the model has vertices that cannot be
// stretched into a position such that the edges connecting them to the rest of the model do not cross
// existing edges (in a 2D plane), then the "containment" of the boundary will be broken, and the
// model segment will be grown to include the region of the model on both sides of the boundary.
rFeatures_EXPORT ObjModel::Ptr segmentSurface( const ObjModel::Ptr model, // Source model
                                         const std::vector<int>& uniqueVertexBoundaryIds,  // Bounding indices
                                         int inUvtxId);    // Start index (front)

// As above, but return that portion of model that ISN'T found from growing that region of the model with inUvtxId
// as the seed vertex. (Gets the complement of the model returned by segmentSurface above).
rFeatures_EXPORT ObjModel::Ptr segmentNotSurface( const ObjModel::Ptr model, // Source model
                                                    const std::vector<int>& uniqueVertexBoundaryIds,  // Bounding indices
                                                    int inUvtxId);    // Start index (front)

rFeatures_EXPORT bool areEndPointsConnected( const ObjModel::Ptr, const std::vector<int>& uvtxs);

// Calculate and return the mean edge distance in the provided model.
rFeatures_EXPORT double calcMeanEdgeDistance( const ObjModel::Ptr);

// Overloaded version also sets the individual edge lengths in the provided vector
// for later calculation of the variance.
rFeatures_EXPORT double calcMeanEdgeDistance( const ObjModel::Ptr, std::vector<double>& elens);


class rFeatures_EXPORT ObjComponentFinder
{
public:
    explicit ObjComponentFinder( const ObjModel::Ptr&);
    ObjModel::Ptr getConnectedComponent( int seedUniqueVertexIndex) const;
private:
    const ObjModel::Ptr _om;
};  // end class


class rFeatures_EXPORT ObjExtentsFinder
{
public:
    explicit ObjExtentsFinder( const ObjModel::Ptr&);

    const cv::Vec3f& getMean() const { return _mean;}
    const cv::Vec3f& getMinExtent() const { return _minExtent;}
    const cv::Vec3f& getMaxExtent() const { return _maxExtent;}
private:
    cv::Vec3f _mean;
    cv::Vec3f _minExtent;
    cv::Vec3f _maxExtent;
};  // end class

}   // end namespace

#endif
