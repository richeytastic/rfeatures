#pragma once
#ifndef RFEATURES_SOBEL_EDGES_EXTRACTOR_H
#define RFEATURES_SOBEL_EDGES_EXTRACTOR_H

#include "FeatureExtractor.h"
using RFeatures::FeatureExtractor;
#include "SobelEdges.h"
using RFeatures::SobelEdges;

/**
 * Sample feature spec for FeatureBuilder:
 * SOBEL {Depth|Light} {1|2} 16
 * See ctor for details.
 */

namespace RFeatures
{

class rFeatures_EXPORT SobelEdgesExtractor : public FeatureExtractor
{
public:
    SobelEdgesExtractor( int deriv, cv::Size fvDims, cv::Mat img);
    SobelEdgesExtractor();
    virtual ~SobelEdgesExtractor();

    virtual void getValidImageTypes( vector<ImageType>&) const;
    virtual string getTypeString() const { return "Sobel";}
    virtual string getParams() const;

    virtual cv::Size getFeatureDims() const { return cv::Size( _fvDims.width*_fvDims.height, 2);}
    virtual cv::Size getMinSamplingDims() const { return cv::Size(3,3);} 

protected:
    virtual FeatureExtractor::Ptr createFromParams( const string&) const;
    virtual FeatureExtractor::Ptr initExtractor( const cv::Mat img) const;
    virtual cv::Mat_<float> extractFV( const cv::Rect rct) const;

private:
    int _deriv;
    cv::Size _fvDims;

    SobelEdges* _sobelx;
    SobelEdges* _sobely;
};  // end class


}   // end namespace

#endif

