#pragma once
#ifndef RFEATURES_PROHOG_EXTRACTOR_H
#define RFEATURES_PROHOG_EXTRACTOR_H

// PRO-HOG {Depth|BGR|CIELab|Grey} {num bins} {direction dependence} {square cell dimensions}

#include "FeatureUtils.h"
#include "FeatureExtractor.h"
using RFeatures::FeatureExtractor;
#include <Convert.h>    // rlib
#include "ProHOG.h"
using RFeatures::ProHOG;
#include <boost/foreach.hpp>


namespace RFeatures
{

class rFeatures_EXPORT ProHOGExtractor : public FeatureExtractor
{
public:
    ProHOGExtractor( int nbins, bool dirDep, cv::Size cellDims, cv::Mat img);
    ProHOGExtractor();
    virtual ~ProHOGExtractor(){}

    virtual void getValidImageTypes( vector<ImageType>&) const;
    virtual string getTypeString() const { return "Pro-HOG";}
    virtual string getParams() const;

    virtual cv::Size getFeatureDims() const { return cv::Size( _cellDims.width*_cellDims.height, 4*_nbins);}
    virtual cv::Size getMinSamplingDims() const { return _cellDims;}

protected:
    virtual FeatureExtractor::Ptr createFromParams( const string&) const;
    virtual FeatureExtractor::Ptr initExtractor( const cv::Mat) const;
    virtual cv::Mat_<float> extractFV( const cv::Rect rct) const;

private:
    int _nbins;
    bool _dirDep;
    cv::Size _cellDims;

    ProHOG::Ptr _prohog;
};  // end class


}   // end namespace

#endif

