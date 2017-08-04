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

/**
 * Detect blobs in images. See:
 * http://en.wikipedia.org/wiki/Maximally_stable_extremal_regions
 **/

#pragma once
#ifndef RFEATURES_MSER_KEYPOINTS_H
#define RFEATURES_MSER_KEYPOINTS_H

#include "KeypointsDetector.h"
using namespace RFeatures;


namespace RFeatures
{

class rFeatures_EXPORT MserKeypoints : public KeypointsDetector
{
public:
    MserKeypoints( const cv::Mat &originalImage,
            int delta,
            int minArea,
            int maxArea,
            double maxVariation,
            double minDiversity,
            int maxEvolution,
            double areaThreshold,
            double minMargin,
            int edgeBlurSize);

    virtual ~MserKeypoints(){}

    virtual Keypoints find() const;

private:
    cv::Mat working_image;
    int delta;
    int minArea;
    int maxArea;
    double maxVar;
    double minDiv;
    int maxEv;
    double areaThsh;
    double minMargin;
    int edgeBlurSz;
};  // end class MserKeypoints

}   // end namespace RFeatures

#endif



