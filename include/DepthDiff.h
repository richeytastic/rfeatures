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
#ifndef RFEATURES_DEPTH_DIFF_H
#define RFEATURES_DEPTH_DIFF_H

#include "FeatureOperator.h"

namespace RFeatures
{

// Four different types of patch point distributions
enum PatchPointType
{
    FOUR_PT,    // 4*3/2 = 6
    FIVE_PT,    // 5*4/2 = 10
    NINE_PT,    // 9*8/2 = 36
    THIRTEEN_PT // 13*12/2 = 78
};  // end enum


int getPointPatchLength( const PatchPointType ddt);



class rFeatures_EXPORT DepthDiff : public RFeatures::FeatureOperator
{
public:
    DepthDiff( const cv::Mat_<float>& img, PatchPointType, float sensitivity=1.0);
    virtual ~DepthDiff(){}

protected:
    virtual cv::Mat_<float> extract( const cv::Rect&) const;

private:
    const cv::Mat_<float> _img;
    const PatchPointType _ppt;
    const float _sensitivity;
};  // end class

}   // end namespace

#endif

