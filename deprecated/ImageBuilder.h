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

#ifndef RFEATURES_IMAGE_BUILDER_H
#define RFEATURES_IMAGE_BUILDER_H

#ifdef _WIN32
#pragma warning( disable : 4251)
#endif

#include <opencv2/opencv.hpp>
#include "PointDataBuilder.h"
using RFeatures::PointDataBuilder;


namespace RFeatures
{

class rFeatures_EXPORT ImageBuilder : public PointDataBuilder
{
public:
    typedef boost::shared_ptr<ImageBuilder> Ptr;

    ImageBuilder( int width, int height);
    static ImageBuilder::Ptr create( int width, int height);

    virtual void setPointCol( int row, int col, byte r, byte g, byte b);
    virtual void reset( int width, int height);

    inline cv::Mat getImage() const { return img;}

private:
    cv::Mat_<cv::Vec3b> img;    // Built 2D image
};  // end class


}   // end namespace

#endif
