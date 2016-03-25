#pragma once
#ifndef RFEATURES_DEPTH_DIFF_EXTRACTOR_H
#define RFEATURES_DEPTH_DIFF_EXTRACTOR_H

/**
 * A feature type that looks at the differences in depth across
 * certain points of a patch. Four different size feature vectors
 * are possible - with increasing sizes encoding more depth detail.
 *
 * Sample feature specification for FeatureBuilder:
 * Depth-Diff Depth FOUR_PT [sensitivity]
 *
 * sensitivity: the local distance (+ve and -ve) difference taken into account. 1 = + or - 1 metre (default).
 *
 * Richard Palmer
 * February 2014
 */

#include "FeatureExtractor.h"
using RFeatures::FeatureExtractor;
#include "DepthDiff.h"
using RFeatures::PatchPointType;
using RFeatures::DepthDiff;


namespace RFeatures
{

class rFeatures_EXPORT DepthDiffExtractor : public FeatureExtractor
{
public:
    DepthDiffExtractor( PatchPointType, float sensitivity, cv::Mat);
    DepthDiffExtractor();
    virtual ~DepthDiffExtractor();

    virtual void getValidImageTypes( vector<ImageType>&) const;
    virtual string getTypeString() const { return "Depth-Diff";}
    virtual string getParams() const;

    virtual cv::Size getFeatureDims() const { return cv::Size(RFeatures::getPointPatchLength( _ppt), 1);}
    virtual cv::Size getMinSamplingDims() const { return cv::Size(2,2);}

protected:
    virtual FeatureExtractor::Ptr createFromParams( const string&) const;
    virtual FeatureExtractor::Ptr initExtractor( const cv::Mat img) const;
    virtual cv::Mat_<float> extractFV( const cv::Rect rct) const;

private:
    PatchPointType _ppt;
    float _sensitivity;

    DepthDiff* _depthDiff;
};  // end class

}   // end namespace

#endif
