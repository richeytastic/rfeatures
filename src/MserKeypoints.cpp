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

#include "MserKeypoints.h"
using namespace RFeatures;


MserKeypoints::MserKeypoints( const cv::Mat &img, int d, int minA, int maxA, double maxV,
    double minD, int maxE, double areaT, double minM, int blSz)
    : KeypointsDetector( img),
    delta(d), minArea(minA), maxArea(maxA), maxVar(maxV), minDiv(minD),
    maxEv(maxE), areaThsh(areaT), minMargin(minM), edgeBlurSz(blSz)
{
    working_image = img.clone();
}   // end ctor



Keypoints MserKeypoints::find() const
{
    cv::MserFeatureDetector fd( delta, minArea, maxArea, maxVar, minDiv,
                                  maxEv, areaThsh, minMargin, edgeBlurSz);
    return KeypointsDetector::detectKeypoints( fd, working_image);
}   // end find
