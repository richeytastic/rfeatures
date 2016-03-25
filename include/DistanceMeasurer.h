#pragma once
#ifndef RFEATURES_DISTANCE_MEASURER_H
#define RFEATURES_DISTANCE_MEASURER_H

#include "ObjModel.h"
#include "DijkstraShortestPathFinder.h"

namespace RFeatures
{

class rFeatures_EXPORT DistanceMeasurer
{
public:
    explicit DistanceMeasurer( const ObjModel::Ptr& om);

    // Given the shortest path between points v0 and v1 over the model, return
    // the point x on that path that maximises d(v0 - x) + d(v1 - x) where d is the L2-norm.
    cv::Vec3f getMaximallyExtrudedPoint( const cv::Vec3f& v0, const cv::Vec3f& v1) const;

    // Same as above but specifying unique vertex IDs.
    cv::Vec3f getMaximallyExtrudedPoint( int uvid0, int uvid1) const;

    // Given a vector of unique vertex IDs, return the index of the element in uvids for
    // the point x that maximises d(v0 - x) + d(v1 - x) where d is the L2-norm.
    int getMaximallyExtrudedPointIndex( const std::vector<int>& uvids) const;

private:
    const ObjModel::Ptr _om;
};  // end class

}   // end namespace

#endif
