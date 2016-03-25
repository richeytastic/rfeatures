#pragma once
#ifndef RFEATURES_IMAGE_HISTOGRAM_H
#define RFEATURES_IMAGE_HISTOGRAM_H

#include "rFeatures_Export.h"
#include <opencv2/opencv.hpp>
typedef unsigned char byte;


namespace RFeatures
{

class rFeatures_EXPORT ImageHistogram
{
public:
    ImageHistogram( const cv::Mat& img, int numBins);

    // Calculate over all channels the histogram for the given range
    // of values for each channel of the image subregion defined by subRect.
    // If subRect is not given, histogram over whole image will be calculated.
    // Returned array will have as many channels as the input image.
    // Zero values in provided mask will ignore corresponding pixels in sub image.
    // (mask must be same size as subRect size!)
    // NB - images with more that 1 channel will result in very large
    // histograms (256*3 for 3 channels) - consider using calcSparseHistogram instead.
    // Returned histogram is NOT normalised!
    cv::Mat calcHistogram( const cv::Rect subRect=cv::Rect(0,0,0,0),
                           cv::Mat_<byte> mask=cv::Mat(),
                           float minVal = 0, float maxVal = 255);

    cv::SparseMat calcSparseHistogram( const cv::Rect subRect=cv::Rect(0,0,0,0),
                           cv::Mat_<byte> mask=cv::Mat(),
                           float minVal = 0, float maxVal = 255);

    //cv::Mat calcHueHistogram( const cv::Rect& subRect, int minSaturation=0);

private:
    const cv::Mat _img;
    const int _nbins;
};  // end class

}   // end namespace

#endif
