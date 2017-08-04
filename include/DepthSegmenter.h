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
#ifndef RFEATURES_DEPTH_SEGMENTER_H
#define RFEATURES_DEPTH_SEGMENTER_H

#include "rFeatures_Export.h"
#include <opencv2/opencv.hpp>
typedef unsigned char byte;


namespace RFeatures
{

class rFeatures_EXPORT DepthSegmenter
{
public:
    // depthLevels: Size of histogram to bin depth values into.
    // inlierFactor: Within the most common depth interval, depth values are retained
    //  that are inlierFactor * stddev within the mean of that depth interval.
    // Range of histogram values (maxRng-minRng) can be overridden by setting minRng and maxRng.
    // Only if minRng < maxRng will these parameters be used to
    // override the min and max as found from the whole of the provided range image.
    // To set the min and max range from an image subregion, use function setMinMaxFromSubRect.
    // Neither minRng or maxRng can be less than zero.
    DepthSegmenter( const cv::Mat_<float> rangeImage, int depthLevels=10, float inlierFactor=2, float minRng=0, float maxRng=0);

    // Calculate a range mask based on the most common occurrence
    // of depth values within the provided sub region. Only values > 0
    // in the provided mask parameter are used (dimensions of this mask
    // must be the same as the subRect).
    cv::Mat_<byte> calcMostCommonDepthMask( const cv::Rect subRect=cv::Rect(0,0,0,0), cv::Mat_<float> mask=cv::Mat_<float>()) const;
    cv::Mat_<byte> operator()( const cv::Rect subRect=cv::Rect(0,0,0,0), cv::Mat_<float> mask=cv::Mat_<float>()) const;

    // If user wishes to use a min, max range for the histogram as derived from
    // a particular subregion of the image (e.g. the same subregion being used to
    // select the common range mask from), user may call this function before
    // calling the common range mask generating functions above.
    void setMinMaxFromSubRect( const cv::Rect& subRect);

    // Return the min,max values used to define the histogram range.
    float getMinRange() const { return _minRng;}
    float getMaxRange() const { return _maxRng;}

    // After a call to the mask generating functions, this returns the statistics from
    // the depth values used to produce the positive (255) values in the mask image.
    void getLastStats( float* lastMinDepth=NULL, float* lastMaxDepth=NULL,
                       float* lastAvgDepth=NULL, float* lastStdDev=NULL) const;

private:
    const cv::Mat_<float> _rngMat;
    float _minRng, _maxRng;
    int _depthLevels;
    float _inlierFactor;

    mutable float _lastMinDepth, _lastMaxDepth, _lastAvgDepth, _lastStdDev;

    int binVal( int* bins, float* means, double v, double vRng) const;
};  // end class

}   // end namespace

#endif
