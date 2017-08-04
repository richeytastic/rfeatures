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
 * In memory decompression of JPEG images.
 *
 * Richard Palmer
 * June 2011
 */

#pragma once
#ifndef RFEATURES_JPEG_DECOMPRESSOR_H
#define RFEATURES_JPEG_DECOMPRESSOR_H

#include <cstdio>
#include <sys/types.h>  // Needed before jpeglib.h since jpeglib.h doesn't define or include some required types!
#include <jpeglib.h>
#include <jerror.h>

#include "rFeatures_Export.h"
#include <opencv2/opencv.hpp>

typedef unsigned char byte;

namespace RFeatures
{

class rFeatures_EXPORT JpegDecompressor
{
public:
    //Decompress a JPEG in memory to a cv::Mat object.
    static cv::Mat decompress( const byte* compressed, size_t nbytes);

private:
    // Convert the JPEG pointed to in memory by jpegPtr of size jpegBytes into
    // raw bytes and store in private array.
    JpegDecompressor( const byte* jpegPtr, size_t jpegBytes);
    ~JpegDecompressor();

    // Decompress the JPEG into an OpenCV Mat image. If decompression couldn't occur
    // for whatever reason, the result will be undefined.
    cv::Mat decompress();

    j_decompress_ptr m_cinfo; // Pointer to jpeg_decompress_struct struct
    struct jpeg_error_mgr *m_jerr;   // Pointer to error manager
    void killCinfo(); // Cleanup
}; // end class

}  // end namespace

#endif
