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
#ifndef RFEATURES_GALL_LEMPITSKY_FEATURE_H
#define RFEATURES_GALL_LEMPITSKY_FEATURE_H

#include "FeatureOperator.h"
typedef unsigned char byte;
#include <vector>
#include "FeatureExceptions.h"
using RFeatures::ImageTypeException;


namespace RFeatures
{

class rFeatures_EXPORT GallLempitskyFeature : public RFeatures::FeatureOperator
{
public:
    // Takes a CV_8UC3
    explicit GallLempitskyFeature( const cv::Mat image);
    virtual ~GallLempitskyFeature();

protected:
    virtual void getSampleChannels( const cv::Rect&, std::vector<cv::Mat>&) const;

private:
    std::vector<IplImage*> _channels;  // Descriptor feature channels (32)
};  // end class

}   // end namespace

#endif




