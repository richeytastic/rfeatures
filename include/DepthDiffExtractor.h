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
#ifndef RFEATURES_DEPTH_DIFF_EXTRACTOR_H
#define RFEATURES_DEPTH_DIFF_EXTRACTOR_H

/**
 * A feature type that looks at the differences in depth across
 * certain points of a patch. Four different size feature vectors
 * are possible - with increasing sizes encoding more depth detail.
 *
 * Sample feature specification for FeatureBuilder:
 * Depth-Diff Depth FOUR_PT [sensitivity]
 *
 * sensitivity: the local distance (+ve and -ve) difference taken into account. 1 = + or - 1 metre (default).
 *
 * Richard Palmer
 * February 2014
 */

#include "FeatureExtractor.h"
using RFeatures::FeatureExtractor;
#include "DepthDiff.h"
using RFeatures::PatchPointType;
using RFeatures::DepthDiff;


namespace RFeatures
{

class rFeatures_EXPORT DepthDiffExtractor : public FeatureExtractor
{
public:
    DepthDiffExtractor( PatchPointType, float sensitivity, cv::Mat);
    DepthDiffExtractor();
    virtual ~DepthDiffExtractor();

    virtual void getValidImageTypes( vector<ImageType>&) const;
    virtual string getTypeString() const { return "Depth-Diff";}
    virtual string getParams() const;

    virtual cv::Size getFeatureDims() const { return cv::Size(RFeatures::getPointPatchLength( _ppt), 1);}
    virtual cv::Size getMinSamplingDims() const { return cv::Size(2,2);}

protected:
    virtual FeatureExtractor::Ptr createFromParams( const string&) const;
    virtual FeatureExtractor::Ptr initExtractor( const cv::Mat img) const;
    virtual cv::Mat_<float> extractFV( const cv::Rect rct) const;

private:
    PatchPointType _ppt;
    float _sensitivity;

    DepthDiff* _depthDiff;
};  // end class

}   // end namespace

#endif
