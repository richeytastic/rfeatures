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

#include "SiftKeypoints.h"
using namespace RFeatures;


SiftKeypoints::SiftKeypoints( const cv::Mat &img, double t, double et)
    : KeypointsDetector( img), threshold(t), edgeThreshold(et)
{
    working_image = img.clone();
}   // end ctor



Keypoints SiftKeypoints::find() const
{
    cv::SiftFeatureDetector fd( threshold, edgeThreshold);
    return KeypointsDetector::detectKeypoints( fd, working_image);
}   // end find



int SiftKeypoints::keypointDrawFlag() const
{
    return cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS;
}   // end keypointDrawFlag
