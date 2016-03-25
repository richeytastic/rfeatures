/**
 * Module that manages ImageTypes and conversions of View::img2d and View::rngImg to the required types.
 */

#pragma once
#ifndef RFEATURES_IMAGE_TYPE_H
#define RFEATURES_IMAGE_TYPE_H

#include <string>
using std::string;
#include <istream>
#include <boost/unordered_set.hpp>

#include "FeatureExtractor.h"
using RFeatures::FeatureExtractor;
#include "ImageTypeEnum.h"
using RFeatures::ImageType;
#include "FeatureUtils.h"
#include "View.h"
using RFeatures::View;


namespace RFeatures
{

// Set the matching image type constants given the input image (i.e. the ImageType constants that
// img can be considered as). Returns the number of matching image types added to the provided vector or < 0
// if no constants from the ImageType enum match the given image.
rFeatures_EXPORT int getMatchingImageTypes( const cv::Mat img, boost::unordered_set<ImageType>& matchingTypes);

// Given img, set imgTypes with the constants of all the ImageTypes that img can be converted to.
// Returns the number of image types that img can be converted to (can be zero).
rFeatures_EXPORT int getConvertibleImageTypes( const cv::Mat img, boost::unordered_set<ImageType>& convertibleTypes);
rFeatures_EXPORT int getConvertibleImageTypes( int cvImgType/*from img.type()*/, boost::unordered_set<ImageType>& convertibleTypes);

// Returns true iff fx can extract from the given type of image (that is, cvImgType images
// can be converted to one of the types that fx is able to process).
rFeatures_EXPORT bool checkFXImageTypeMismatch( const FeatureExtractor* fx, const cv::Mat img);
rFeatures_EXPORT bool checkFXImageTypeMismatch( const FeatureExtractor::Ptr fx, const cv::Mat img);

rFeatures_EXPORT cv::Mat createImageType( ImageType, cv::Mat img) throw (ImageTypeException);

// If rct is not default, subregion of view will be parsed only, otherwise the whole view is parsed.
rFeatures_EXPORT cv::Mat createImageType( ImageType, const View::Ptr v, cv::Rect rct=cv::Rect(0,0,0,0));

rFeatures_EXPORT ImageType parseImageType( std::istream& ss) throw (ImageTypeException);

rFeatures_EXPORT string toString( ImageType imgType) throw (ImageTypeException);


}   // end namespace

#endif


