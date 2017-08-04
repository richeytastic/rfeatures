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
 * Tools for extracting Pro-HOG feature vectors from images.
 * Richard Palmer
 * Jan 2013
 */

#pragma once
#ifndef RFEATURES_PRO_HOG_TOOLS_H
#define RFEATURES_PRO_HOG_TOOLS_H

#include <vector>
using std::vector;
#include <opencv2/opencv.hpp>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include "ProHOG.h"

typedef unsigned int uint;


namespace RFeatures
{

// Given a Pro-HOG feature matrix (as a cv::Mat), calculate and return
// its internal differences between each of the cells. Only positive differences
// are used. Results in a feature vector having length M*R*C*(R*C-1)/2 where M is the number
// of histogram bins in the provided Pro-HOG feature matrix (e.g. 9) and R and C are
// the number of rows and columns respectively in the provided feature matrix.
rFeatures_EXPORT cv::Mat calcInternalDiffs( const cv::Mat &phog, bool appendOriginal=false);


// Multi-threaded extraction of Pro-HOG feature vectors from a vector of examples.
class rFeatures_EXPORT BatchProHOGExtractor
{
public:
    // Extracts ProHOG features in a multithreaded batch process.
    BatchProHOGExtractor( const vector<cv::Mat> &objectExamples,  // Original (bounded) instances
                          int numBins, bool dirDep, const cv::Size &cellDims,    // ProHOG extraction parameters
                          bool useInternalDiffs=false,      // Use ProHOG differences or not
                          bool appendOrig=true); // If using internal diffs, whether to append original ProHOGs

    // Sets the extracted feature vectors to be row vectors (number of rows == number of examples),
    // with each row having N*R*C entries (with N=4*nbins).
    void extract( vector<cv::Mat_<float> > &phogs);  // Single threaded
    void extract_mt( vector<cv::Mat_<float> > &phogs);   // Multi-threaded

    // Sets the feature vector matrices as returned from the raw Pro-HOG class (CV_64FC(4*nbins))
    void extract( vector<cv::Mat> &phogs);  // Single threaded
    void extract_mt( vector<cv::Mat> &phogs);   // Multi-threaded

private:
    const vector<cv::Mat> &_imgs;
    const uint _nbins;
    const bool _dirDep;
    const cv::Size _cellDims;
    bool _useSpatialDiffs;
    bool _appendOrig;
};  // end class

}   // end namespace


#endif
