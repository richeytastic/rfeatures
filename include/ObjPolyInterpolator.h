#ifndef RFEATURES_OBJ_POLY_INTERPOLATOR_H
#define RFEATURES_OBJ_POLY_INTERPOLATOR_H

/**
 * Used with ObjModelRemesher to correctly interpolate points on input mesh triangles.
 */

#include "ObjModelFastMarcher.h"

namespace RFeatures
{

class ObjPolyInterpolator
{
public:
    ObjPolyInterpolator( const ObjModel::Ptr inputModel,
                         const ObjModel::Ptr outputModel,
                         const boost::unordered_map<int,int> &nearestSources,
                         const boost::unordered_map<int, boost::unordered_map<int, double> > &itimes);

    // Given vidx on the input model, find a suitable ObjPoly to interpolate
    // over (with vidx as one of its vertices), returning the interpolated position.
    cv::Vec3f interpolate( int vidx) const;

    // Given two external points X and Y and known arrival times of X and Y at A and B, 
    // calculate the time at which X and Y meet along a line segment with A and B as endpoints.
    static double calcLambda( double txa, double txb, double tya, double tyb);

    // Given the value of lambda above, get the point along the line segment between vA and vB.
    // This is calculated simply as (1.0 - lambda)*vA + lambda*vB.
    static cv::Vec3d calcLambdaPoint( double lambda, const cv::Vec3d& vA, const cv::Vec3d& vB);

private:
    const ObjModel::Ptr _inmod;
    const ObjModel::Ptr _outmod;
    const boost::unordered_map<int,int> &_nearestSources;
    const boost::unordered_map<int, boost::unordered_map<int, double> > &_itimes;

    void addInterpolatedPosition( cv::Vec3d&,int,int,int,int) const;

    void printDebug( int,int) const;
    double calcTriangleArea( int,int,int) const;
};  // end class

}   // end namespace

#endif
