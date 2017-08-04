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
#ifndef RFEATURES_FAST_HOG_EXTRACTOR_H
#define RFEATURES_FAST_HOG_EXTRACTOR_H

/**
 * Sample feature spec for FeatureBuilder:
 * FAST-HOG {DEPTH|COLOUR} 7 9 5 {WITH-MAX|WITHOUT-MAX} {WITH-ANGLES|WITHOUT-ANGLES}
 * Refer to constructor below for parameter details.
 */

#include "FeatureExtractor.h"
using RFeatures::FeatureExtractor;
#include "FastHOG.h"
using RFeatures::FastHOG;


namespace RFeatures
{

class rFeatures_EXPORT FastHOGExtractor : public FeatureExtractor
{
public:
    FastHOGExtractor( int nbins, cv::Size pxlWin, cv::Size featVecDims, cv::Mat img);
    FastHOGExtractor();
    virtual ~FastHOGExtractor();

    virtual void getValidImageTypes( vector<ImageType>&) const;
    virtual string getTypeString() const { return "Fast-HOG";}
    virtual string getParams() const;

    virtual cv::Size getFeatureDims() const { return cv::Size( _fvDims.width*_fvDims.height*( _nbins+2), 1);}
    virtual cv::Size getMinSamplingDims() const { return _pxlWin;}

protected:
    virtual FeatureExtractor::Ptr createFromParams( const string&) const;
    virtual FeatureExtractor::Ptr initExtractor( const cv::Mat img) const;
    virtual cv::Mat_<float> extractFV( const cv::Rect rct) const;

private:
    // _nbins: size of histogram to bin pixel contrast gradients to
    // _pxlWin: grouping size of local HOG region
    // _fvDims: fixed scale that every rectangular extract is sampled at before conversion to feature vector.
    int _nbins;
    cv::Size _pxlWin;
    cv::Size _fvDims;

    FastHOG* _fhog;
};  // end class


}   // end namespace

#endif

