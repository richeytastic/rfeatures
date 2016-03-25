#include "CannyOperator.h"
using RFeatures::CannyOperator;
#include <algorithm>
#include <cassert>


CannyOperator::CannyOperator( const cv::Mat &img, int lowT, int highT, const cv::Size fvDims)
    : FeatureOperator( img.size(), fvDims)
{
    assert( img.channels() == 1);
    // Set thresholds
    lowT = std::max<int>(0,lowT);
    highT = std::max<int>(0,highT);
    cv::Mat timg;
    cv::Canny( img, timg, lowT, highT);
    if ( timg.depth() != CV_32F)
        timg.convertTo( _cimg, CV_32F);
    else
        _cimg = timg;
}   // end ctor



void CannyOperator::getSamplingImages( vector<cv::Mat>& simgs) const
{
    simgs.push_back( _cimg);
}   // end simgs
