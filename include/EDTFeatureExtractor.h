#pragma once
#ifndef RFEATURES_EDT_FEATURE_EXTRACTOR_H
#define RFEATURES_EDT_FEATURE_EXTRACTOR_H

#include "FeatureExtractor.h"
using RFeatures::FeatureExtractor;
#include "EDTFeature.h"
using RFeatures::EDTFeature;

namespace RFeatures
{

class rFeatures_EXPORT EDTFeatureExtractor : public FeatureExtractor
{
public:
    EDTFeatureExtractor( int lowCT, int highCT, cv::Size fvDims, cv::Mat img);
    EDTFeatureExtractor();
    virtual ~EDTFeatureExtractor();

    virtual void getValidImageTypes( vector<ImageType>&) const;
    virtual string getTypeString() const { return "EDT";}
    virtual string getParams() const;

    virtual cv::Size getFeatureDims() const { return cv::Size( _fvDims.width*_fvDims.height, 1);}
    virtual cv::Size getMinSamplingDims() const { return cv::Size(2,2);}

    // img: Must be single channel
    // Returns an inverted binary edge map
    static cv::Mat_<byte> createBinaryEdgeMap( const cv::Mat& img, int lowCT, int highCT);

protected:
    virtual FeatureExtractor::Ptr createFromParams( const string&) const;
    virtual FeatureExtractor::Ptr initExtractor( const cv::Mat img) const;
    virtual cv::Mat_<float> extractFV( const cv::Rect rct) const;

private:
    int _lowCT, _highCT;
    cv::Size _fvDims;
    EDTFeature* _edt;
};  // end class

}   // end namespace

#endif
