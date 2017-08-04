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
#ifndef RFEATURES_GRADIENT_EXTRACTOR_H
#define RFEATURES_GRADIENT_EXTRACTOR_H

// GRD {BGR|CIELab|Depth|EDT|Grey|Light} {nbins} {gradient direction sensitivity} {blockDims}
// e.g. GRD BGR 9 false 7
// Note: +1 blockdims = celldims

#include "FeatureUtils.h"
#include "FeatureExtractor.h"
using RFeatures::FeatureExtractor;
#include "View.h"
#include <Convert.h>    // rlib


namespace RFeatures
{

class rFeatures_EXPORT GradientExtractor : public FeatureExtractor
{
public:
    GradientExtractor( int nbins, bool gradientDirectionSensitive, int blockDims, cv::Mat img);
    GradientExtractor();
    virtual ~GradientExtractor(){}

    virtual void getValidImageTypes( vector<ImageType>&) const;
    virtual string getTypeString() const { return "GRD";}
    virtual string getParams() const;

    virtual cv::Size getFeatureDims() const;
    virtual cv::Size getMinSamplingDims() const;

protected:
    virtual FeatureExtractor::Ptr createFromParams( const string&) const;
    virtual FeatureExtractor::Ptr initExtractor( const cv::Mat) const;
    virtual cv::Mat_<float> extractFV( const cv::Rect) const;

private:
    int _nbins;
    int _blockDims;
    double _binRads;
    std::vector<cv::Mat_<double> > _gradIntImg;
    cv::Mat_<double> _gradSumIntImg;

    void extractHistograms( const cv::Rect& rct, cv::Mat_<double>& cellfvs, double* bsums) const;
    cv::Mat_<float> normaliseCells( const cv::Mat_<double>& cellfvs, const double* bsums) const;

    void setGradientBinRange( bool dirSensitive);
};  // end class


}   // end namespace

#endif

