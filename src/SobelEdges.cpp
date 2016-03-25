#include "SobelEdges.h"
using RFeatures::SobelEdges;
#include <cassert>
#include <iostream>
#include <algorithm>
#include <cmath>


// Ensure all values in s are between 0 and 1 inclusive
void checkVals( const cv::Mat_<float>& s)
{
    bool bad = true;
    for ( int i = 0; i < s.rows; ++i)
    {
        const float* row = s.ptr<float>(i);
        for ( int j = 0; j < s.cols; ++j)
        {
            if ( row[j] < 0.0 || row[j] > 1.0)
            {
                std::cerr << "[ERROR] SobelEdges::checkVals: invalid feature value = " << row[j] << std::endl;
                bad = false;
            }   // end if
        }   // end for
    }   // end for

    assert(bad);
}   // end checkVals



SobelEdges::SobelEdges( const cv::Mat_<float>& img, int xd, int yd, const cv::Size fvDims)
    : RFeatures::FeatureOperator( img.size(), fvDims)
{
    assert( yd == 2 && xd == 0 || yd == 0 && xd == 2 || yd == 1 && xd == 0 || yd == 0 && xd == 1);

    const int ks = 3;   // 3x3 kernel size
    cv::Mat_<float> sobImg;

    cv::Sobel( img, sobImg, CV_32F, xd, yd, ks, 0.25);
    _s = cv::abs(sobImg);

#ifndef NDEBUG
    checkVals( _s);
#endif
}   // end ctor



void SobelEdges::getSampleChannels( const cv::Rect& rct, vector<cv::Mat>& simgs) const
{
    simgs.push_back(_s(rct));
}   // end getSampleChannels
