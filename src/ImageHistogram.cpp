/************************************************************************
 * Copyright (C) 2017 Richard Palmer
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 ************************************************************************/

#include <ImageHistogram.h>
using RFeatures::ImageHistogram;
#include <cassert>
#include <vector>


ImageHistogram::ImageHistogram( const cv::Mat& img, int nBins) : _img(img), _nbins(nBins)
{
    assert( img.depth() == CV_8U);
}   // end ctor




cv::Mat ImageHistogram::calcHistogram( const cv::Rect subRect, cv::Mat_<byte> mask, float minVal, float maxVal)
{
    cv::Mat subImg = _img;
    if ( subRect.area() > 0)
        subImg = _img(subRect);
    if ( mask.empty())
        mask = cv::Mat_<byte>::ones( subImg.size());
    assert( mask.rows == subImg.rows && mask.cols == subImg.cols);

    const int nChannels = subImg.channels();

    const float minMaxRange[2] = {minVal, maxVal+1}; // Min and max values (max is exclusive)
    const float** ranges = (const float**)cv::fastMalloc(nChannels * sizeof(const float*));
    int *histSize = (int*)cv::fastMalloc(nChannels * sizeof(int));
    int *channels = (int*)cv::fastMalloc(nChannels * sizeof(int));
    for ( int i = 0; i < nChannels; ++i)
    {
        histSize[i] = _nbins;   // Size of histogram for each channel
        channels[i] = i;        // Channels to create histograms for
        ranges[i] = minMaxRange;// Min,max range for each channel
    }   // end for

    cv::Mat hist; // Will be resulting histogram
    cv::calcHist( &subImg, 1/*# images*/, channels, mask, hist, nChannels, histSize, ranges);

    cv::fastFree(ranges);
    cv::fastFree(histSize);
    cv::fastFree(channels);

    return hist;
}   // end calcHistogram



cv::SparseMat ImageHistogram::calcSparseHistogram( const cv::Rect subRect, cv::Mat_<byte> mask, float minVal, float maxVal)
{
    cv::Mat subImg = _img;
    if ( subRect.area() > 0)
        subImg = _img(subRect);
    if ( mask.empty())
        mask = cv::Mat_<byte>::ones( subImg.size());
    assert( mask.rows == subImg.rows && mask.cols == subImg.cols);

    const int nChannels = subImg.channels();

    const float minMaxRange[2] = {minVal, maxVal+1}; // Min and max values (max is exclusive)
    const float** ranges = (const float**)cv::fastMalloc(nChannels * sizeof(const float*));
    int *histSize = (int*)cv::fastMalloc(nChannels * sizeof(int));
    int *channels = (int*)cv::fastMalloc(nChannels * sizeof(int));
    for ( int i = 0; i < nChannels; ++i)
    {
        histSize[i] = _nbins;   // Size of histogram for each channel
        channels[i] = i;        // Channels to create histograms for
        ranges[i] = minMaxRange;// Min,max range for each channel
    }   // end for

    cv::SparseMat hist; // Will be resulting histogram
    cv::calcHist( &subImg, 1/*# images*/, channels, mask, hist, nChannels, histSize, ranges);

    cv::fastFree(ranges);
    cv::fastFree(histSize);
    cv::fastFree(channels);

    return hist;
}   // end calcSparseHistogram




/*
cv::MatND getHueHistogram( const cv::Mat &img, int minSat)
{
    cv::MatND hist;

    // Convert to HSV colour space
    cv::Mat hsv;
    cv::cvtColor( img, hsv, CV_BGR2HSV);
    
    // Mask to be used (or not)
    cv::Mat mask;
    if ( minSat > 0)
    {
        // Split the 3 channels into 3 images
        std::vector<cv::Mat> v;
        cv::split(hsv,v);
        // Mask the low saturated pixels
        cv::threshold( v[1], mask, minSat, 255, cv::THRESH_BINARY);
    }   // end if

    // Prepare arguments for a 1D hue histogram
    float hranges[2] = {0.0, 180.0};   // Pixel value ranges
    const float* ranges[1] = {hranges};
    const int channels[1] = {0};      // The hue channel
    const int histSize[1] = {180};    // Only 180 separate values for hue used by OpenCV

    // Compute histogram
    cv::calcHist( &hsv,
            1,          // Histogram of 1 image only
            channels,   // The channel used
            mask,       // The saturation mask
            hist,       // The result
            1,          // Dimensionality of histogram
            histSize,   // Number of bins
            ranges     // Pixel value range
            );

    return hist;
}   // end getHueHistogram
*/
