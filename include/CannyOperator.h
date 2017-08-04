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
#ifndef RFEATURES_CANNY_OPERATOR_H
#define RFEATURES_CANNY_OPERATOR_H

#include "FeatureOperator.h"
using RFeatures::FeatureOperator;
#include "rFeatures_Export.h"


namespace RFeatures
{

class rFeatures_EXPORT CannyOperator : public FeatureOperator
{
public:
    // img: CV_8UC1
    CannyOperator( const cv::Mat& img, int lowThreshold, int highThreshold, const cv::Size fvDims=cv::Size(0,0));
    virtual ~CannyOperator(){}

    cv::Mat_<float> getEdgeImage() const { return _cimg;}

protected:
    virtual void getSamplingImages( vector<cv::Mat>& simgs) const;

private:
    cv::Mat_<float> _cimg;  // Image to work on
};  // end class

}   // end namespace

#endif
