/**
 * Build a range image and an integral image of range.
 * 
 * Richard Palmer
 * September 2012
 */

#pragma once
#ifndef RFEATURES_RANGE_BUILDER
#define RFEATURES_RANGE_BUILDER

#include <opencv2/opencv.hpp>
#include "PointDataBuilder.h"
using RFeatures::PointDataBuilder;
#include "IntegralImage.h"
using RFeatures::IntegralImage;


namespace RFeatures
{

class rFeatures_EXPORT RangeBuilder : public PointDataBuilder
{
public:
    typedef boost::shared_ptr<RangeBuilder> Ptr;

    RangeBuilder( int width, int height);
    static RangeBuilder::Ptr create( int width, int height);

    // Non-positive range values are not allowed! Range values < 0 are set
    // to zero and no range value is allowed to be greater than MAX_ALLOWED_RANGE
    // (which is equal to FLT_MAX). Can be called following IntegralImager::setValue
    // build creation rules.
    virtual void setPointRange( int row, int col, double rng);
    virtual void reset( int width, int height);

    inline cv::Mat_<float> getRangeImage() const { return rngData_;}
    inline IntegralImage<double>::Ptr getIntegralRangeImage() const { return rngIntImg_;}
    inline float getMaxRange() const { return maxRange_;}

    static const float MAX_ALLOWED_RANGE;

private:
    cv::Mat_<float> rngData_;    // Raw range data
    IntegralImage<double>::Ptr rngIntImg_;   // Integral image of range values
    float maxRange_;         // Max range value in rngData
};  // end class RangeBuilder

}   // end namespace RFeatures

#endif
