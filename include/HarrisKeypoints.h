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
#ifndef RFEATURES_HARRIS_KEYPOINTS_H
#define RFEATURES_HARRIS_KEYPOINTS_H

#include "rFeatures_Export.h"
#include "KeypointsDetector.h"
using RFeatures::KeypointsDetector;


namespace RFeatures
{

class rFeatures_EXPORT HarrisKeypoints : public KeypointsDetector
{
public:
    HarrisKeypoints( const cv::Mat &originalImage, int maxCorners, double quality, int minDist);
    virtual ~HarrisKeypoints(){}

    virtual Keypoints find() const;

private:
    cv::Mat working_image;
    int maxCorners;
    double quality;
    int minDist;
};  // end class HarrisKeypoints

}   // end namespace RFeatures

#endif
