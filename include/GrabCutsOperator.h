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

#ifndef RFEATURES_GRAB_CUTS_OPERATOR_H
#define RFEATURES_GRAB_CUTS_OPERATOR_H

#ifdef _WIN32
#pragma warning( disable : 4251)
#endif

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
