/**
 * Compare images for similarity based on their histograms.
 */

#pragma once
#ifndef RLIB_IMAGE_COMPARATOR_H
#define RLIB_IMAGE_COMPARATOR_H

#include <ImageProcess.h>
#include <ColourHistogram.h>
using namespace RFeatures;
#include <opencv2/opencv.hpp>


namespace RFeatures
{

class ImageComparator
{
public:
    ImageComparator( int factor, const cv::Mat &reference_img);

    // Returns a number between 0 and 1 where numbers closer to
    // 1 are more "similar" to the reference image.
    double operator()( const cv::Mat &comparison_img);

private:
    cv::MatND refH;
    cv::MatND inputH;

    ColourHistogram hist;
    cv::Size imgsz;  // Size of the images (must be equal)
    int div;

    void setColourReduction( int factor);
    void setReferenceImage( const cv::Mat &img);
};  // end class ImageComparator

}   // end namespace RFeatures

#endif
