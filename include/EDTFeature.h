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
#ifndef RFEATURES_EDT_FEATURE_H
#define RFEATURES_EDT_FEATURE_H

#include "FeatureUtils.h"
#include "FeatureOperator.h"
#include "DistanceTransform.h"

namespace RFeatures
{

class rFeatures_EXPORT EDTFeature : public RFeatures::FeatureOperator
{
public:
    // img: Must be a binary valued CV_8UC1
    EDTFeature( const cv::Mat_<byte> img, const cv::Size fvDims=cv::Size(0,0));
    virtual ~EDTFeature(){}

protected:
    virtual cv::Mat_<float> extract( const cv::Rect&) const;
    virtual void getSampleChannels( const cv::Rect&, vector<cv::Mat>&) const;

private:
    cv::Mat_<double> _dtimg;
    cv::Mat_<double> _iimg;
};  // end class

}   // end namespace

#endif
