#pragma once
#ifndef RFEATURES_HISTOGRAM_H
#define RFEATURES_HISTOGRAM_H

#include "FeatureUtils.h"
#include <vector>

namespace RFeatures
{

class rFeatures_EXPORT Histogram
{
public:
    explicit Histogram( const cv::Mat_<byte>& m);

    void drawHistograms( const cv::Size& sz) const;

private:
    std::vector<int> _hist; // Histogram of pixel values
    std::vector<int> _chist;    // Cummulative histogram
};  // end class

}   // end namespace


#endif
