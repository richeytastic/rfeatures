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

#pragma once
#ifndef RFEATURES_LOCAL_BINARY_PATTERN_EXTRACTOR_H
#define RFEATURES_LOCAL_BINARY_PATTERN_EXTRACTOR_H

/**
 * Local binary patterns.
 * Sample feature specification for FeatureBuilder:
 * LBP Depth
 *
 * Richard Palmer
 * September 2014
 */

#include "FeatureExtractor.h"
using RFeatures::FeatureExtractor;
#include "LocalBinaryPattern.h"
using RFeatures::LocalBinaryPattern;


namespace RFeatures
{

class rFeatures_EXPORT LocalBinaryPatternExtractor : public FeatureExtractor
{
public:
    explicit LocalBinaryPatternExtractor( cv::Mat); // Takes single channel images only
    LocalBinaryPatternExtractor();
    virtual ~LocalBinaryPatternExtractor();

    virtual void getValidImageTypes( vector<ImageType>&) const;
    virtual string getTypeString() const { return "LBP";}
    virtual string getParams() const { return "";}  // No params!

    virtual cv::Size getFeatureDims() const { return cv::Size( 1, 1);}
    virtual cv::Size getMinSamplingDims() const { return cv::Size(3,3);}

protected:
    virtual FeatureExtractor::Ptr createFromParams( const string&) const;
    virtual FeatureExtractor::Ptr initExtractor( const cv::Mat img) const;
    virtual cv::Mat_<float> extractFV( const cv::Rect rct) const;

private:
    LocalBinaryPattern* _lbp;
};  // end class

}   // end namespace

#endif
