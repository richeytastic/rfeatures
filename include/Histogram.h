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
#ifndef RFEATURES_HISTOGRAM_H
#define RFEATURES_HISTOGRAM_H

#include "FeatureUtils.h"
#include <vector>

namespace RFeatures
{

class rFeatures_EXPORT Histogram
{
public:
    explicit Histogram( const cv::Mat_<byte>& m);

    void drawHistograms( const cv::Size& sz) const;

private:
    std::vector<int> _hist; // Histogram of pixel values
    std::vector<int> _chist;    // Cummulative histogram
};  // end class

}   // end namespace


#endif
