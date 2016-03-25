#pragma once
#ifndef RFEATURES_SOBEL_EDGES_H
#define RFEATURES_SOBEL_EDGES_H

#include "FeatureOperator.h"

namespace RFeatures
{

class rFeatures_EXPORT SobelEdges : public RFeatures::FeatureOperator
{
public:
    // img: CV_32FC1 with values in [0,1]
    // dx: 1 or 2 (1st or 2nd degree sobel edges) for x (vertical edges)
    // dy: 1 or 2 (1st or 2nd degree sobel edges) for y (horizontal edges)
    // If dx == 1 then dy == 0
    // If dy == 1 then dx == 0
    // if dx == 2 then dy == 0
    // if dy == 2 then dx == 0
    // fvDims: sampling feature vector size to fix dimensions of feature vector (if not used, no sampling done)
    SobelEdges( const cv::Mat_<float>& img, int dx, int dy, const cv::Size fvDims=cv::Size(0,0));
    virtual ~SobelEdges(){}

protected:
    virtual void getSampleChannels( const cv::Rect&, vector<cv::Mat>&) const;

private:
    cv::Mat_<float> _s; // Sobel edge map
};  // end class

}   // end namespace

#endif


