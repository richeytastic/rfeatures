#pragma once
#ifndef RFEATURES_GRADIENT_EXTRACTOR_H
#define RFEATURES_GRADIENT_EXTRACTOR_H

// GRD {BGR|CIELab|Depth|EDT|Grey|Light} {nbins} {gradient direction sensitivity} {blockDims}
// e.g. GRD BGR 9 false 7
// Note: +1 blockdims = celldims

#include "FeatureUtils.h"
#include "FeatureExtractor.h"
using RFeatures::FeatureExtractor;
#include "View.h"
#include <Convert.h>    // rlib


namespace RFeatures
{

class rFeatures_EXPORT GradientExtractor : public FeatureExtractor
{
public:
    GradientExtractor( int nbins, bool gradientDirectionSensitive, int blockDims, cv::Mat img);
    GradientExtractor();
    virtual ~GradientExtractor(){}

    virtual void getValidImageTypes( vector<ImageType>&) const;
    virtual string getTypeString() const { return "GRD";}
    virtual string getParams() const;

    virtual cv::Size getFeatureDims() const;
    virtual cv::Size getMinSamplingDims() const;

protected:
    virtual FeatureExtractor::Ptr createFromParams( const string&) const;
    virtual FeatureExtractor::Ptr initExtractor( const cv::Mat) const;
    virtual cv::Mat_<float> extractFV( const cv::Rect) const;

private:
    int _nbins;
    int _blockDims;
    double _binRads;
    std::vector<cv::Mat_<double> > _gradIntImg;
    cv::Mat_<double> _gradSumIntImg;

    void extractHistograms( const cv::Rect& rct, cv::Mat_<double>& cellfvs, double* bsums) const;
    cv::Mat_<float> normaliseCells( const cv::Mat_<double>& cellfvs, const double* bsums) const;

    void setGradientBinRange( bool dirSensitive);
};  // end class


}   // end namespace

#endif

