#pragma once
#ifndef RFEATURES_CONTENT_FINDER_H
#define RFEATURES_CONTENT_FINDER_H

#include "rFeatures_Export.h"
#include <opencv2/opencv.hpp>

namespace RFeatures
{

class rFeatures_EXPORT ContentFinder
{
public:
    ContentFinder();

    void setThreshold( float t);
    inline float getThreshold() const { return m_threshold;}

    void setHistogram( const cv::MatND &histogram);

    cv::Mat find( const cv::Mat &img, float minVal, float maxVal, int *channels, int dim);

private:
    float m_hranges[2];
    const float* m_ranges[3];
    int m_channels[3];
    float m_threshold;
    cv::MatND m_histogram;
};  // end class ContentFinder

}   // end namespace RFeatures

#endif
