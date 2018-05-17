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

#ifndef RFEATURES_IMAGE_TYPE_H
#define RFEATURES_IMAGE_TYPE_H

#include "FeatureExtractor.h"
#include "ImageTypeEnum.h"
#include "FeatureUtils.h"
#include <unordered_set>
#include <istream>
#include <string>

namespace RFeatures {

// Set the matching image type constants given the input image (i.e. the ImageType constants that
// img can be considered as). Returns the number of matching image types added to the provided vector or < 0
// if no constants from the ImageType enum match the given image.
rFeatures_EXPORT int getMatchingImageTypes( const cv::Mat img, std::unordered_set<ImageType>& matchingTypes);

// Given img, set imgTypes with the constants of all the ImageTypes that img can be converted to.
// Returns the number of image types that img can be converted to (can be zero).
rFeatures_EXPORT int getConvertibleImageTypes( const cv::Mat img, std::unordered_set<ImageType>& convertibleTypes);
rFeatures_EXPORT int getConvertibleImageTypes( int cvImgType/*from img.type()*/, std::unordered_set<ImageType>& convertibleTypes);

// Returns true iff fx can extract from the given type of image (that is, cvImgType images
// can be converted to one of the types that fx is able to process).
rFeatures_EXPORT bool checkFXImageTypeMismatch( const FeatureExtractor* fx, const cv::Mat img);
rFeatures_EXPORT bool checkFXImageTypeMismatch( const FeatureExtractor::Ptr fx, const cv::Mat img);

rFeatures_EXPORT cv::Mat createImageType( ImageType, cv::Mat img) throw (ImageTypeException);

rFeatures_EXPORT ImageType parseImageType( std::istream& ss) throw (ImageTypeException);

rFeatures_EXPORT std::string toString( ImageType imgType) throw (ImageTypeException);

}   // end namespace

#endif
