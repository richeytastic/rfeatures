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
#ifndef RFEATURES_CONTENT_FINDER_H
#define RFEATURES_CONTENT_FINDER_H

#include "RFeatures.h"

namespace RFeatures
{

class rFeatures_EXPORT ContentFinder
{
public:
    ContentFinder();

    void setThreshold( float t);
    inline float getThreshold() const { return m_threshold;}

    void setHistogram( const cv::MatND &histogram);

    cv::Mat find( const cv::Mat &img, float minVal, float maxVal, int *channels, int dim);

private:
    float m_hranges[2];
    const float* m_ranges[3];
    int m_channels[3];
    float m_threshold;
    cv::MatND m_histogram;
};  // end class ContentFinder

}   // end namespace RFeatures

#endif
