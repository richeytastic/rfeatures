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
#ifndef RFEATURES_PROHOG_EXTRACTOR_H
#define RFEATURES_PROHOG_EXTRACTOR_H

// PRO-HOG {Depth|BGR|CIELab|Grey} {num bins} {direction dependence} {square cell dimensions}

#include "FeatureUtils.h"
#include "FeatureExtractor.h"
using RFeatures::FeatureExtractor;
#include <Convert.h>    // rlib
#include "ProHOG.h"
using RFeatures::ProHOG;
#include <boost/foreach.hpp>


namespace RFeatures
{

class rFeatures_EXPORT ProHOGExtractor : public FeatureExtractor
{
public:
    ProHOGExtractor( int nbins, bool dirDep, cv::Size cellDims, cv::Mat img);
    ProHOGExtractor();
    virtual ~ProHOGExtractor(){}

    virtual void getValidImageTypes( vector<ImageType>&) const;
    virtual string getTypeString() const { return "Pro-HOG";}
    virtual string getParams() const;

    virtual cv::Size getFeatureDims() const { return cv::Size( _cellDims.width*_cellDims.height, 4*_nbins);}
    virtual cv::Size getMinSamplingDims() const { return _cellDims;}

protected:
    virtual FeatureExtractor::Ptr createFromParams( const string&) const;
    virtual FeatureExtractor::Ptr initExtractor( const cv::Mat) const;
    virtual cv::Mat_<float> extractFV( const cv::Rect rct) const;

private:
    int _nbins;
    bool _dirDep;
    cv::Size _cellDims;

    ProHOG::Ptr _prohog;
};  // end class


}   // end namespace

#endif

