/**
 * Provide generic functionality for creation of gradient maps.
 * Gradients are soft binned into according to the gradient's
 * "position" in the target bin.
 * Gradients are also binned into the bins of adjacent pixels.
 *
 * Richard Palmer
 * September 2012
 */

#pragma once
#ifndef RFEATURES_GRADIENTS_BUILDER
#define RFEATURES_GRADIENTS_BUILDER

#include <opencv2/opencv.hpp>
#include "FeatureUtils.h"
#include "rFeatures_Export.h"


namespace RFeatures
{

// Type must ALWAYS be a cv::Vec type (even for single channel images).
struct GradientsBuilder
{

// Set the nbins gradient images of img.
template <typename T>
static rFeatures_EXPORT void calculateGradients( const cv::Mat_<T>& img,
                                        int nbins, bool dirDep, std::vector<cv::Mat_<double> >& grads);

// Calculate the gradient (theta) and the magnitude (returned) of any arbitrary
// pixel indexed by ridx,cidx in image img. Gradient calculated is the maximum
// over all channels. Type of image MUST be a cv::Vec type (even if single channel).
template <typename T>
static rFeatures_EXPORT double calcGradient( const cv::Mat_<T> &img, int ridx, int cidx, double &theta);


static rFeatures_EXPORT void setPixelGradient( int row, int col, double mag, double theta,
                                               double binRads, std::vector<cv::Mat_<double> >& gradients);

};  // end class


#include "template/GradientsBuilder_template.h"

}   // end namespace

#endif



