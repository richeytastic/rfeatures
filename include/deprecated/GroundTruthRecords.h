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
 * Holds extracts and features of ground truth examples from .pano data.
 * Also provides feature vectors taken from panoramas that don't contain
 * the object class of interest.
 *
 * Richard Palmer
 * April 2015
 */

#ifndef RFEATURES_GROUND_TRUTH_RECORDS_H
#define RFEATURES_GROUND_TRUTH_RECORDS_H

#include "DataTools.h"
#include "FeatureUtils.h"
#include "FeatureExtractor.h"
#include "AdaptiveDepthPatchScanner.h"  // For PatchRanger
#include "ImageType.h"
#include <Convert.h>    // RLIB
#include <Random.h>     // RLIB
#include <iostream>
#include <unordered_map>
#include <unordered_set>

namespace RFeatures {

struct ViewGT
{
    int id;
    cv::Size win;   // Dimensions of the view
    std::string vstr;   // String identifier
    std::vector<cv::Rect> grndTrth; // Ground truth boxes
    View::Ptr view;
    void getExtracts( std::vector<cv::Mat>& exs, ImageType, bool useHorzFlipped) const;
};  // end struct


class GroundTruthRecords
{
public:
    // Use cacheViews==true (default) to retain views from the gtfile in memory once loaded.
    // (Can use a lot of memory for very large datasets).
    // Set viewsAsImages to true if working with a directory of image dumped panoramas
    // (each panorama containing FACE_colour.png and FACE_depth.png for FACE in {FRONT,LEFT,REAR,RIGHT}).
    GroundTruthRecords( const std::string& panoDir, bool viewsAsImages, bool cacheViews=true);
    bool areViewsCached() const;

    // Load the positive extracts data. Only loads extracts having pixel dimensions at least as large
    // as those given in minPxlDims. Returns the number of extracts loaded or -1 on error.
    int loadPosExtracts( const std::string& gtfile, const cv::Size& minPxlDims, std::ostream* debugInfo=NULL);
    const ViewGT getView( int viewId);
    int getNumViews() const;

    // Call this function to retrieve the IDs of views with overlapping ground truth boxes.
    const std::vector<int>& getGroundTruthOverlaps() const;

    // Returns the average dimensions of the positive extracts as queried using the range images.
    // This means that all views must be loaded in (which is why this function isn't const).
    // Set measureFromBase true if depth measurements are to be taken from the base of each of
    // the ground truth boxes, otherwise depth measurements are taken from the centre of the boxes.
    cv::Size2f getRealWorldMeanObjectDims( bool measureFromBase=false);

    // Creates a pool of negative extracts of size negMultiple x positives
    // taken from views that don't have any of the positive examples. Returns the number created.
    // The extracts can be returned by calling getNegs().
    int loadNegExtracts( int negMultitple, bool useHorzFlipped, ImageType, std::ostream* debugInfo=NULL);

    // Since loading and returning the whole extracts can be a burden on memory, this function
    // allows the user to extract feature vectors using the given feature extractor. Upon loading
    // each negative extract, the feature vector is extracted and placed into this objects _negExtracts member.
    // Returns the number of feature vectors created. The function getNegs() will return the feature vectors.
    int loadNegFeatureVectors( int negMultiple, bool useHorzFlipped, const RFeatures::FeatureExtractor::Ptr fx, std::ostream* debugInfo=NULL);

    // Returns either the raw extracts (if loadNegExtracts() was called) or the feature vectors
    // (if loadNegFeatureVectors() was called).
    const std::vector<cv::Mat>& getNegs() const;

private:
    const std::string _panoDir;
    const bool _viewsAsImages;
    const bool _cacheViews;
    int _totalPosExs;

    std::vector<int> _posOverlaps;  // Upon loading, set with views having overlapping ground truth boxes
    std::vector<ViewGT> _views;
    std::vector<cv::Mat> _negExtracts;  // Negative extracts or feature vectors

    std::unordered_map<std::string, std::unordered_set<int> > _posPanoViews;    // Pano ID strings with views having positive examples
    std::unordered_map<std::string, std::unordered_set<int> > _negPanoViews;    // Pano ID strings with views for possible negatives
    std::vector<std::string> _negPanoIds;   // All pano IDs that can be used to fetch random negatives

    int loadNegs( int, bool useHorzFlipped, ImageType, const RFeatures::FeatureExtractor::Ptr, std::ostream* debugInfo=NULL);
};  // end class

}   // end namespace

#endif



