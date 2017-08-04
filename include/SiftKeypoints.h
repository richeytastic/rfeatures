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
#ifndef RFEATURES_SIFT_KEYPOINTS_H
#define RFEATURES_SIFT_KEYPOINTS_H

#include "KeypointsDetector.h"
using namespace RFeatures;


namespace RFeatures
{

class SiftKeypoints : public KeypointsDetector
{
public:
    SiftKeypoints( const cv::Mat &originalImage,
            double threshold,
            double edgeThreshold);

    virtual ~SiftKeypoints(){}

    virtual Keypoints find() const;

protected:
    virtual int keypointDrawFlag() const;

private:
    cv::Mat working_image;
    double threshold;
    double edgeThreshold;
};  // end class SiftKeypoints

}   // end namespace RFeatures

#endif


