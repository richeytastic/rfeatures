#pragma once
#ifndef RFEATURES_GALL_LEMPITSKY_FEATURE_EXTRACTOR_H
#define RFEATURES_GALL_LEMPITSKY_FEATURE_EXTRACTOR_H

#include "FeatureExtractor.h"
using RFeatures::FeatureExtractor;
#include "GallLempitskyFeature.h"
using RFeatures::GallLempitskyFeature;


namespace RFeatures
{

class rFeatures_EXPORT GallLempitskyFeatureExtractor : public FeatureExtractor
{
public:
    explicit GallLempitskyFeatureExtractor( cv::Mat m); // Accepts CV_8UC3 only
    GallLempitskyFeatureExtractor();
    virtual ~GallLempitskyFeatureExtractor();

    virtual void getValidImageTypes( vector<ImageType>&) const;
    virtual string getTypeString() const { return "Gall-Lempitsky";}
    virtual string getParams() const;
    virtual cv::Size getFeatureDims() const { return cv::Size(256, 32);}    // 32 features over 16x16 image patches
    virtual cv::Size getMinSamplingDims() const { return cv::Size( 3, 3);}

protected:
    virtual FeatureExtractor::Ptr createFromParams( const string&) const;
    virtual FeatureExtractor::Ptr initExtractor( const cv::Mat img) const;
    virtual cv::Mat_<float> extractFV( const cv::Rect rct) const;

private:
    GallLempitskyFeature* _glf;
};  // end class


}   // end namespace

#endif

