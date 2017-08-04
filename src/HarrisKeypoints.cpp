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

#include "HarrisKeypoints.h"
using namespace RFeatures;


HarrisKeypoints::HarrisKeypoints( const cv::Mat &img, int maxC, double q, int minD)
    : KeypointsDetector( img)
{
    if ( img.channels() != 1)
        cv::cvtColor( img, working_image, CV_BGR2GRAY);
    else
        working_image = img.clone();

    maxCorners = maxC;
    if ( maxCorners < 1)
        maxCorners = 1;
    quality = q;
    minDist = minD;
    if ( minDist < 1)
        minDist = 1;
}   // end ctor



Keypoints HarrisKeypoints::find() const
{
    cv::GoodFeaturesToTrackDetector fd( maxCorners, quality, minDist);
    return KeypointsDetector::detectKeypoints( fd, working_image);
}   // end find
