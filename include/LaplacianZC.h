#pragma once
#ifndef RFEATURES_LAPLACIAN_ZC_H
#define RFEATURES_LAPLACIAN_ZC_H

#include "rFeatures_Export.h"
#include <opencv2/opencv.hpp>

namespace RFeatures
{

class rFeatures_EXPORT LaplacianZC
{
public:
    LaplacianZC( int apertureSize);

    void setAperture( int apertureSize);

    // Compute the floating point laplacian matrix.
    cv::Mat computeLaplacian( const cv::Mat &image);

    // Get the laplacian result in an 8-bit image for display.
    cv::Mat getLaplacianImage( const cv::Mat &image);

    // Get a binary image of the zero-crossings. If the product of
    // two adjacent pixels is less than threshold, the zero-crossing will be ignored.
    cv::Mat getZeroCrossings( const cv::Mat &image, float threshold=1.0);

private:
    int aperture;   // Aperture size of laplacian kernel
};  // end class LaplacianZC

}   // end namespace RFeatures

#endif
