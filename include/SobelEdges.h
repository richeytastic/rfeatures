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
#ifndef RFEATURES_SOBEL_EDGES_H
#define RFEATURES_SOBEL_EDGES_H

#include "FeatureOperator.h"

namespace RFeatures
{

class rFeatures_EXPORT SobelEdges : public RFeatures::FeatureOperator
{
public:
    // img: CV_32FC1 with values in [0,1]
    // dx: 1 or 2 (1st or 2nd degree sobel edges) for x (vertical edges)
    // dy: 1 or 2 (1st or 2nd degree sobel edges) for y (horizontal edges)
    // If dx == 1 then dy == 0
    // If dy == 1 then dx == 0
    // if dx == 2 then dy == 0
    // if dy == 2 then dx == 0
    // fvDims: sampling feature vector size to fix dimensions of feature vector (if not used, no sampling done)
    SobelEdges( const cv::Mat_<float>& img, int dx, int dy, const cv::Size fvDims=cv::Size(0,0));
    virtual ~SobelEdges(){}

protected:
    virtual void getSampleChannels( const cv::Rect&, vector<cv::Mat>&) const;

private:
    cv::Mat_<float> _s; // Sobel edge map
};  // end class

}   // end namespace

#endif


