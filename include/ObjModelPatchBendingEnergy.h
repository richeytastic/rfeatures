#ifndef RFEATURES_OBJ_MODEL_PATCH_BENDING_ENERGY_H
#define RFEATURES_OBJ_MODEL_PATCH_BENDING_ENERGY_H

#include "ObjModel.h"

namespace RFeatures
{

class rFeatures_EXPORT ObjModelPatchBendingEnergy
{
public:
    ObjModelPatchBendingEnergy( const ObjModel::Ptr m, const ObjModel::Ptr n);

    // Calculate the bending energy required to deform patch p (taken from m) to
    // patch q (taken from n) using the 2D thin-plate spline model.
    // Invalid (negative) value returned if p.size() != q.size().
    double operator()( const IntSet& p, const IntSet& q) const;

    // As above, but as single column, N row matrices where p.rows == q.rows.
    // Invalid (negative) value returned if p.rows != q.rows.
    static double calc( const cv::Mat_<cv::Vec3f>& p, const cv::Mat_<cv::Vec3f>& q);
    static double calcE( const cv::Mat_<cv::Vec3f>& p, const cv::Mat_<cv::Vec3f>& q);   // Uses Eigen

private:
    const ObjModel::Ptr _m0, _m1;
};  // end class

}   // end namespace

#endif
