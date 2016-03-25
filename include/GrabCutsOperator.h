#pragma once
#ifndef RFEATURES_GRAB_CUTS_OPERATOR_H
#define RFEATURES_GRAB_CUTS_OPERATOR_H

#include "rFeatures_Export.h"
#include <opencv2/opencv.hpp>

namespace RFeatures
{

class rFeatures_EXPORT GrabCutsOperator
{
public:
    // fgArea is a rectangle containing the area of img likely to contain foreground objects. All
    // pixels outside of fgArea are marked as certain background.
    // iterations is the number of iterations before the algorithm should return.
    GrabCutsOperator( const cv::Mat &img, const cv::Rect &fgArea, int iterations);
    virtual ~GrabCutsOperator(){}

    // Returns the segmented image.
    // Segmented image pixels can be one of 4 values:
    // cv::GC_BGD - definite background
    // cv::GC_FGD - definite foreground
    // cv::GC_PR_BGD - probably background
    // cv::GC_PR_FGD - probably foreground
    cv::Mat findSegmentation() const;

    // Returns a binary image of the foreground with white pixels being foreground
    // and black pixels being background.
    cv::Mat getBinaryForeground( const cv::Mat &segmentMatrix) const;

    // Returns a copy of the original image with all non-foreground objects masked as black.
    cv::Mat getImageForeground( const cv::Mat &segmentMatrix) const;

private:
    const cv::Rect fgArea;    // Inital area set to contain all foreground objects
    const int iters;  // Number of iterations
    cv::Mat image;  // The image we're applying segmentation to
};  // end class GrabCutsOperator

}   // end namespace RFeatures

#endif
