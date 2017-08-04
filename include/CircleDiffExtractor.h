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
#ifndef RFEATURES_CIRCLE_DIFF_EXTRACTOR_H
#define RFEATURES_CIRCLE_DIFF_EXTRACTOR_H

#include "FeatureExtractor.h"
using RFeatures::FeatureExtractor;
#include "CircleDiff.h"
using RFeatures::CircleDiff;

/**
 * Sample feature spec for FeatureBuilder:
 * Circle-Diff Depth 8 
 * See constructor below for details.
 */

namespace RFeatures
{

class rFeatures_EXPORT CircleDiffExtractor : public FeatureExtractor
{
public:
    CircleDiffExtractor( int numPoints, cv::Mat m);
    CircleDiffExtractor();
    virtual ~CircleDiffExtractor();

    virtual void getValidImageTypes( vector<ImageType>&) const;
    virtual string getTypeString() const { return "Circle-Diff";}
    virtual string getParams() const;

    virtual cv::Size getFeatureDims() const { return cv::Size( 3*_nps, 1);};
    virtual cv::Size getMinSamplingDims() const { return cv::Size(2,2);}

protected:
    virtual FeatureExtractor::Ptr createFromParams( const string&) const;
    virtual FeatureExtractor::Ptr initExtractor( const cv::Mat img) const;
    virtual cv::Mat_<float> extractFV( const cv::Rect rct) const;

private:
    int _nps;
    CircleDiff* _cdiff;
};  // end class


}   // end namespace

#endif

