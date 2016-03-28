/**
 * Build range gradient image from range data provided by a PointDataReader.
 * 
 * Richard Palmer
 * September 2012
 */

#pragma once
#ifndef RFEATURES_RANGE_GRADIENTS_BUILDER
#define RFEATURES_RANGE_GRADIENTS_BUILDER

#include <opencv2/opencv.hpp>
#include "PointDataBuilder.h"
using RFeatures::PointDataBuilder;
#include "GradientsBuilder.h"
using RFeatures::GradientsBuilder;
typedef unsigned char byte;


namespace RFeatures
{

class rFeatures_EXPORT RangeGradientsBuilder : public PointDataBuilder
{
public:
    typedef boost::shared_ptr<RangeGradientsBuilder> Ptr;

    // Static constructors
    static RangeGradientsBuilder::Ptr create( const cv::Mat_<double> &img,
            int nbins, bool dirDep=true, bool spatialSmooth=true);
    static RangeGradientsBuilder::Ptr create( const cv::Mat_<float> &img,
            int nbins, bool dirDep=true, bool spatialSmooth=true);
    static RangeGradientsBuilder::Ptr create( const cv::Mat_<byte> &img,
            int nbins, bool dirDep=true, bool spatialSmooth=true);
    static RangeGradientsBuilder::Ptr create(
            int nbins, bool dirDep=true, bool spatialSmooth=true);

    // Constructors
    RangeGradientsBuilder( const cv::Mat_<double> &img,
            int nbins, bool dirDep=true, bool spatialSmooth=true);
    RangeGradientsBuilder( const cv::Mat_<float> &img,
            int nbins, bool dirDep=true, bool spatialSmooth=true);
    RangeGradientsBuilder( const cv::Mat_<byte> &img,
            int nbins, bool dirDep=true, bool spatialSmooth=true);

    // The following constructor may be used with the subsequent two functions
    // to create the gradient map incrementally.
    RangeGradientsBuilder( int nbins, bool dirDep=true, bool spatialSmooth=true);
    virtual ~RangeGradientsBuilder(){}

    virtual void reset( int width, int height);

    // Range values < 0 are set to zero. Range values > RangeBuilder::MAX_ALLOWED_RANGE
    // are set to RangeBuilder::MAX_ALLOWED_RANGE.
    virtual void setPointRange( int row, int col, double rng);

    IntegralImage<double>::Ptr getIntegralGradients() const { return grads.getIntegralGradients();}

    inline int getNumBins() const { return grads.getNumBins();}
    inline bool isDirectionDependent() const { return grads.isDirectionDependent();}
    inline bool isSpatiallySmoothed() const { return grads.isSpatiallySmoothed();}
    inline double getBinRads() const { return grads.getBinRads();}

private:
    bool byPoint;   // True iff creating incrementally
    int nbins;
    bool dirDep;
    bool spatialSmooth;
    GradientsBuilder<cv::Vec<double, 1> > grads;
};  // end class RangeGradientsBuilder

}   // end namespace RFeatures

#endif
