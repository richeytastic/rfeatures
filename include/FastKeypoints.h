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
#ifndef RFEATURES_FAST_KEYPOINTS_H
#define RFEATURES_FAST_KEYPOINTS_H

#include "rFeatures_Export.h"
#include "KeypointsDetector.h"
using namespace RFeatures;


namespace RFeatures
{

class rFeatures_EXPORT FastKeypoints : public KeypointsDetector
{
public:
    FastKeypoints( const cv::Mat &originalImage, double threshold);
    virtual ~FastKeypoints(){}

    virtual Keypoints find() const;

private:
    cv::Mat working_image;
    double threshold;
};  // end class FastKeypoints

}   // end namespace RFeatures

#endif



