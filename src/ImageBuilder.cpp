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

#include "ImageBuilder.h"
using RFeatures::ImageBuilder;



ImageBuilder::Ptr ImageBuilder::create( int width, int height)
{
    return ImageBuilder::Ptr( new ImageBuilder( width, height));
}   // end create



ImageBuilder::ImageBuilder( int width, int height)
{
    reset( width, height);
}   // end ctor



void ImageBuilder::setPointCol( int row, int col, byte r, byte g, byte b)
{
    cv::Vec3b &v = img(row, col);
    v[2] = r; v[1] = g; v[0] = b; // Reverse indices for OpenCV
}   // end setPointCol



void ImageBuilder::reset( int width, int height)
{
    img.create( height, width);
}   // end reset
