#pragma once
#ifndef RFEATURES_HOG_EXTRACTOR_H
#define RFEATURES_HOG_EXTRACTOR_H

/**
 * Sample feature spec for FeatureBuilder (nbins, num blocks wide, num blocks high, cell pixel dims)
 * HOG {BGR|GREY} 18 false 15 9 8
 * Refer to constructor below for parameter details.
 */

#include "FeatureExtractor.h"
using RFeatures::FeatureExtractor;
#include <Convert.h>    // rlib


namespace RFeatures
{

class rFeatures_EXPORT HOGExtractor : public FeatureExtractor
{
public:
    HOGExtractor( int nbins, bool dirDep, cv::Size blockGridDims, int cellPxlDims, cv::Mat img);
    HOGExtractor();
    virtual ~HOGExtractor();

    virtual void getValidImageTypes( vector<ImageType>&) const;
    virtual string getTypeString() const { return "HOG";}
    virtual string getParams() const;

    virtual cv::Size getFeatureDims() const { return cv::Size( 4*_nbins * _bdims.width * _bdims.height, 1);}
    virtual cv::Size getMinSamplingDims() const { return cv::Size( _bdims.width+1, _bdims.height+1);}   // 2x2 pixel

protected:
    virtual FeatureExtractor::Ptr createFromParams( const string&) const;
    virtual FeatureExtractor::Ptr initExtractor( const cv::Mat img) const;
    virtual cv::Mat_<float> extractFV( const cv::Rect rct) const;

private:
    int _nbins;
    bool _dirDep;
    cv::Size _bdims;
    int _cellPxls;

    cv::HOGDescriptor* _hog;
    cv::Mat _img;   // CV_8UC1
};  // end class


}   // end namespace

#endif

