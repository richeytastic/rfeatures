#pragma once
#ifndef RFEATURES_LOCAL_BINARY_PATTERN_EXTRACTOR_H
#define RFEATURES_LOCAL_BINARY_PATTERN_EXTRACTOR_H

/**
 * Local binary patterns.
 * Sample feature specification for FeatureBuilder:
 * LBP Depth
 *
 * Richard Palmer
 * September 2014
 */

#include "FeatureExtractor.h"
using RFeatures::FeatureExtractor;
#include "LocalBinaryPattern.h"
using RFeatures::LocalBinaryPattern;


namespace RFeatures
{

class rFeatures_EXPORT LocalBinaryPatternExtractor : public FeatureExtractor
{
public:
    explicit LocalBinaryPatternExtractor( cv::Mat); // Takes single channel images only
    LocalBinaryPatternExtractor();
    virtual ~LocalBinaryPatternExtractor();

    virtual void getValidImageTypes( vector<ImageType>&) const;
    virtual string getTypeString() const { return "LBP";}
    virtual string getParams() const { return "";}  // No params!

    virtual cv::Size getFeatureDims() const { return cv::Size( 1, 1);}
    virtual cv::Size getMinSamplingDims() const { return cv::Size(3,3);}

protected:
    virtual FeatureExtractor::Ptr createFromParams( const string&) const;
    virtual FeatureExtractor::Ptr initExtractor( const cv::Mat img) const;
    virtual cv::Mat_<float> extractFV( const cv::Rect rct) const;

private:
    LocalBinaryPattern* _lbp;
};  // end class

}   // end namespace

#endif
