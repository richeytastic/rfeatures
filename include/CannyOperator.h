#pragma once
#ifndef RFEATURES_CANNY_OPERATOR_H
#define RFEATURES_CANNY_OPERATOR_H

#include "FeatureOperator.h"
using RFeatures::FeatureOperator;
#include "rFeatures_Export.h"


namespace RFeatures
{

class rFeatures_EXPORT CannyOperator : public FeatureOperator
{
public:
    // img: CV_8UC1
    CannyOperator( const cv::Mat& img, int lowThreshold, int highThreshold, const cv::Size fvDims=cv::Size(0,0));
    virtual ~CannyOperator(){}

    cv::Mat_<float> getEdgeImage() const { return _cimg;}

protected:
    virtual void getSamplingImages( vector<cv::Mat>& simgs) const;

private:
    cv::Mat_<float> _cimg;  // Image to work on
};  // end class

}   // end namespace

#endif
