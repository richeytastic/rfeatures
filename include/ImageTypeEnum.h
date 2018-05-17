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

#ifndef RFEATURES_IMAGE_TYPE_ENUM_H
#define RFEATURES_IMAGE_TYPE_ENUM_H

#include "rFeatures_Export.h"
#include <functional>

#define MAX_RANGE_M 100

namespace RFeatures {

enum rFeatures_EXPORT ImageType
{
    BGR,    // CV_8UC3
            // The following three image types require CV_8UC3 for conversion in createImageType
    Grey,   // Single channel grey value (CV_8UC1) (i.e. flattened BGR)
    Light,  // Single channel grey value (CV_32FC1) with values in [0,1]
    CIELab, // 3 channel CIE Lab (CV_32FC3)

            // The following two image types require CV_32FC1 for conversion in createImageType
    Depth,  // Single channel depth map with values in [0,1] (CV_32FC1)
    EDT,    // Euclidean distance map on range with values in [0,1]
};  // end enum

}   // end namespace

// Treat enum as int for hashing.
namespace std {
template <> struct hash<RFeatures::ImageType>
{
    size_t operator()( const RFeatures::ImageType& v) const { return hash<int>()((int)v);}
};  // end struct

}   // end namespace std


#endif
