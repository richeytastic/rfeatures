#pragma once
#ifndef RFEATURES_HISTOGRAM_1D_H
#define RFEATURES_HISTOGRAM_1D_H

#include "rFeatures_Export.h"
#include <opencv2/opencv.hpp>

namespace RFeatures
{

class rFeatures_EXPORT Histogram1D
{
public:
    Histogram1D();

    cv::MatND getHistogram( const cv::Mat &img);
    cv::Mat getHistogramImage( const cv::Mat &img);

    // Contrast stretch ignoring histogram bins with less than minVal pixels.
    static cv::Mat stretch( const cv::Mat &img, int minVal=0);

    static cv::Mat applyLookUp( const cv::Mat &img, const cv::Mat &lookup);

    static cv::Mat equalise( const cv::Mat &img);

private:
    int m_histSize[1]; // Number of bins
    float m_hranges[2];   // Min and max pixel values
    const float* m_ranges[1];
    int m_channels[1];  // Only 1 channel used here
};  // end class Histogram1D

}   // end namespace RFeatures

#endif
