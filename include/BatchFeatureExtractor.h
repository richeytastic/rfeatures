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

/**
 * Batch process lots of cv::Mat extracts into cv::Mat_<float> row vectors.
 */

#pragma once
#ifndef RFEATURES_BATCH_FEATURE_EXTRACTOR_H
#define RFEATURES_BATCH_FEATURE_EXTRACTOR_H

#include "FeatureUtils.h"
#include "FeatureExtractor.h"
#include "FeatureExceptions.h"
#include "ImageType.h"
#include <vector>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include "CpuTimer.h"   // rlib

namespace RFeatures
{

class rFeatures_EXPORT BatchFeatureExtractor
{
public:
    // Set the image extracts to extract features from, which are resized by the given scale factor prior
    // to being sent to the feature extractor. No extract is scaled below the minimum sampling dimensions for the feature extractor.
    BatchFeatureExtractor( FeatureExtractor::Ptr fx, const std::vector<cv::Mat>& extracts, double scaleFact=1);

    // Extract features from many different locations over the same image.
    // The image is first pre-processed according to the feature extractor being used.
    // The image is then resized by the given factor before preprocessing, and all given extract rectangles
    // are also resized by the given scale factor to ensure scale congruence.
    BatchFeatureExtractor( FeatureExtractor::Ptr fx, const cv::Mat& img, const std::vector<cv::Rect>& exts, double scaleFact=1);

    // Process and return feature vectors corresponding to each of the set extracts.
    const std::vector<cv::Mat_<float> >& extract();

    double msecs() const;    // Returns the number of milliseconds taken to extract

private:
    FeatureExtractor::Ptr _fx;
    const double _scaleFactor;  // Used for resizing the image prior to pre-processing and the extracts (keeps aspect ratio intact).
    const std::vector<cv::Mat>* _extracts; // Image extracts to FX pre-process and extract.
    const std::vector<cv::Rect>* _rextracts; // Sub regions of an already pre-processed FX to be extracted.
    std::vector<cv::Mat_<float> > _fvs; // Extracted feature vectors
    double _msecs;

    void extractRectFeatures( int, int);
    void extractMatFeatures( int, int);
};  // end class

}   // end namespace

#endif
