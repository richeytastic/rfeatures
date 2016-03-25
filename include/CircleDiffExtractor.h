#pragma once
#ifndef RFEATURES_CIRCLE_DIFF_EXTRACTOR_H
#define RFEATURES_CIRCLE_DIFF_EXTRACTOR_H

#include "FeatureExtractor.h"
using RFeatures::FeatureExtractor;
#include "CircleDiff.h"
using RFeatures::CircleDiff;

/**
 * Sample feature spec for FeatureBuilder:
 * Circle-Diff Depth 8 
 * See constructor below for details.
 */

namespace RFeatures
{

class rFeatures_EXPORT CircleDiffExtractor : public FeatureExtractor
{
public:
    CircleDiffExtractor( int numPoints, cv::Mat m);
    CircleDiffExtractor();
    virtual ~CircleDiffExtractor();

    virtual void getValidImageTypes( vector<ImageType>&) const;
    virtual string getTypeString() const { return "Circle-Diff";}
    virtual string getParams() const;

    virtual cv::Size getFeatureDims() const { return cv::Size( 3*_nps, 1);};
    virtual cv::Size getMinSamplingDims() const { return cv::Size(2,2);}

protected:
    virtual FeatureExtractor::Ptr createFromParams( const string&) const;
    virtual FeatureExtractor::Ptr initExtractor( const cv::Mat img) const;
    virtual cv::Mat_<float> extractFV( const cv::Rect rct) const;

private:
    int _nps;
    CircleDiff* _cdiff;
};  // end class


}   // end namespace

#endif

