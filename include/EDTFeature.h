#pragma once
#ifndef RFEATURES_EDT_FEATURE_H
#define RFEATURES_EDT_FEATURE_H

#include "FeatureUtils.h"
#include "FeatureOperator.h"
#include "DistanceTransform.h"

namespace RFeatures
{

class rFeatures_EXPORT EDTFeature : public RFeatures::FeatureOperator
{
public:
    // img: Must be a binary valued CV_8UC1
    EDTFeature( const cv::Mat_<byte> img, const cv::Size fvDims=cv::Size(0,0));
    virtual ~EDTFeature(){}

protected:
    virtual cv::Mat_<float> extract( const cv::Rect&) const;
    virtual void getSampleChannels( const cv::Rect&, vector<cv::Mat>&) const;

private:
    cv::Mat_<double> _dtimg;
    cv::Mat_<double> _iimg;
};  // end class

}   // end namespace

#endif
