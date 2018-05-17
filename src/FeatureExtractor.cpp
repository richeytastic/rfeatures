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

#include <FeatureExtractor.h>
#include <ImageType.h>
#include <iostream>
#include <iomanip>
using RFeatures::FeatureExtractor;


FeatureExtractor::FeatureExtractor( cv::Size imgSz) : _imgSz(imgSz), _imgTypeSet(false), _fixedDims(0,0) {}

FeatureExtractor::~FeatureExtractor(){}

cv::Size FeatureExtractor::getMinSamplingDims() const { return cv::Size(1,1);}  // Base size can be overridden


std::string FeatureExtractor::getConstructString() const
{
    assert( _imgTypeSet);
    std::ostringstream oss;
    oss << this->getTypeString() << " " << RFeatures::toString( _imgType) << " " << this->getParams();
    return oss.str();
}   // end getConstructString


cv::Size FeatureExtractor::getImageSize() const { return _imgSz;}


void FeatureExtractor::setImageType( ImageType imgType)
{
    _imgType = imgType;
    _imgTypeSet = true;
}   // end setImageType


ImageType FeatureExtractor::getImageType() const
{
    assert( _imgTypeSet);
    return _imgType;
}   // end getImageType


const cv::Mat& FeatureExtractor::getRawImage() const { return _rawImg;}


void FeatureExtractor::setFixedExtractSize( const cv::Size& fixedDims) throw (RFeatures::ImageSizeException)
{
    const cv::Size& minDims = this->getMinSamplingDims();
    if ( fixedDims.width < minDims.width || fixedDims.height < minDims.height)
        throw RFeatures::ImageSizeException( "[EXCEPTION] FeatureExtractor::setFixedExtractSize: Requested resize dimensions of extracts smaller than minimum allowed feature extractor sampling dimensions!");
    _fixedDims = fixedDims;
}   // end setFixedExtractSize

const cv::Size& FeatureExtractor::getFixedExtractSize() const { return _fixedDims;}

cv::Mat_<float> FeatureExtractor::extract( const cv::Rect rct, const cv::Size& fixedDims) const throw (RFeatures::ImageSizeException)
{
#ifndef NDEBUG
    std::cerr << "Processing rct at fixed resolution" << std::endl;
    assert( RFeatures::isContained( _rawImg, rct));
    const cv::Size minDims = this->getMinSamplingDims();
    if ( fixedDims.width < minDims.width || fixedDims.height < minDims.height)
        throw RFeatures::ImageSizeException( "[ERROR] FeatureExtractor::extract(cv::Rect, cv::Size): FX specifies minimum dimensions larger than required fixed extract dimensions!");
#endif

    cv::Mat ex = _rawImg(rct);  // Get the required subregion from the raw image
    cv::Mat rex;
    cv::resize( ex, rex, fixedDims);
    FeatureExtractor::Ptr fx = preProcess( rex);
    fx->_fixedDims.width = fx->_fixedDims.height = 0;   // Don't want to recurse!
    return fx->extract( cv::Rect(0,0,rex.cols,rex.rows));   // Extract from the whole subregion
}   // end extract


cv::Mat_<float> FeatureExtractor::extract( const cv::Rect rct) const throw (RFeatures::ImageSizeException)
{
    assert( !_rawImg.empty());
    assert( RFeatures::isContained( _rawImg, rct));
#ifndef NDEBUG
    const cv::Size minDims = this->getMinSamplingDims();
    if ( rct.width < minDims.width || rct.height < minDims.height)
        throw RFeatures::ImageSizeException( "[ERROR] FeatureExtractor::extract(cv::Rect): FX specifies minimum dimensions larger than rct dimensions!");
#endif

    if ( _fixedDims.width > 0)  // Process the raw image extract at fixed dimensions
        return extract( rct, _fixedDims);

    return this->extractFV( rct); // Pure virtual child
}   // end extract


cv::Mat_<float> FeatureExtractor::extract() const
{
    return extract( cv::Rect(0,0,_imgSz.width, _imgSz.height));    // Rectangle for whole image
}   // end extract


cv::Mat_<float> FeatureExtractor::extract( const cv::Mat& ex) const throw (ExtractorTypeException, ImageTypeException)
{
    const FeatureExtractor::Ptr fx = preProcess(ex);    // Will throw if this feature extractor doesn't have the right image type set
    return fx->extract();
}   // end extract


FeatureExtractor::Ptr FeatureExtractor::createNew( const std::string& params) const throw (ExtractorTypeException)
{
    if ( !_imgTypeSet)
        throw ExtractorTypeException( "[EXCEPTION] FeatureExtractor::createNew: Must set image type before constructing new feature!");

    FeatureExtractor::Ptr fx = this->createFromParams( params);
    if ( fx == NULL)
        throw ExtractorTypeException( "[EXCEPTION] FeatureExtractor::createNew: Unable to create new feature extractor from params " + params);

    fx->setImageType( _imgType);
    return fx;
}   // end createNew


FeatureExtractor::Ptr FeatureExtractor::preProcess( const cv::Mat img) const throw (ExtractorTypeException, ImageTypeException)
{
    if ( !_imgTypeSet)
        throw ExtractorTypeException( "[EXCEPTION] FeatureExtractor::preProcess: No ImageType yet set in " + getTypeString() + " extractor!");

    // The image type set in this feature extractor must match the type of incoming image
    if ( !RFeatures::checkFXImageTypeMismatch( this, img))
        throw RFeatures::ImageTypeException( "[EXCEPTION] FeatureExtractor::preProcess: Given image type does not match this feature extractor!");

    FeatureExtractor::Ptr newFX = this->initExtractor( img);
    newFX->setImageType( _imgType);
    newFX->_rawImg = img;
    newFX->_fixedDims = _fixedDims;
    return newFX;
}   // end preProcess

/*
FeatureExtractor::Ptr FeatureExtractor::preProcess( const View::Ptr view) const
{
    ImageType imgType = getImageType();
    const cv::Mat img = RFeatures::createImageType( imgType, view);
    // Error checking not required
    FeatureExtractor::Ptr newFX = this->initExtractor(img);
    newFX->setImageType( imgType);
    newFX->_rawImg = img;
    newFX->_fixedDims = _fixedDims;
    return newFX;
}   // end preProcess
*/
