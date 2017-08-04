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
#ifndef RFEATURES_ADAPTIVE_DEPTH_MIN_FILTER_H
#define RFEATURES_ADAPTIVE_DEPTH_MIN_FILTER_H

#include "rFeatures_Export.h"
#include "AdaptiveDepthPatchScanner.h"

namespace RFeatures
{

class rFeatures_EXPORT AdaptiveDepthMinFilter
{
public:
    AdaptiveDepthMinFilter( const cv::Mat_<float> depthMap, const cv::Size2f& realPatchSize);

    cv::Mat_<float> filter( float minRange=0, float maxRange=FLT_MAX);

private:
    const cv::Mat_<float> _rngImg;
    const cv::Size2f _rpatchSz;
};  // end class

}   // end namespace

#endif
