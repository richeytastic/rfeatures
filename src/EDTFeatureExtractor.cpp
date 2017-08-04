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

#include "EDTFeatureExtractor.h"
using RFeatures::EDTFeatureExtractor;
#include <sstream>
#include "FeatureUtils.h"


EDTFeatureExtractor::EDTFeatureExtractor( int lowCT, int highCT, cv::Size fvd, cv::Mat img)
    : FeatureExtractor(img.size()), _lowCT(lowCT), _highCT(highCT), _fvDims(fvd), _edt(NULL)
{
    const cv::Mat_<byte> bimg = EDTFeatureExtractor::createBinaryEdgeMap( img, lowCT, highCT);
    _edt = new EDTFeature( bimg, fvd);
}  // end ctor


EDTFeatureExtractor::EDTFeatureExtractor()
    : FeatureExtractor(), _lowCT(0), _highCT(0), _fvDims(0,0), _edt(NULL)
{}  // end ctor


EDTFeatureExtractor::~EDTFeatureExtractor()
{
    if ( _edt != NULL)
        delete _edt;
}   // end dtor



void EDTFeatureExtractor::getValidImageTypes( vector<ImageType>& vimgTypes) const
{
    vimgTypes.push_back(Grey);
    vimgTypes.push_back(Light);
    vimgTypes.push_back(Depth);
}   // end getValidImageTypes



string EDTFeatureExtractor::getParams() const
{
    std::ostringstream ss;
    ss << _lowCT << " " << _highCT << " " << _fvDims.width;
    return ss.str();
}   // end getParams



cv::Mat_<byte> EDTFeatureExtractor::createBinaryEdgeMap( const cv::Mat& img, int lowCT, int highCT)
{
    assert( img.channels() == 1);

    // Get scaling parameter for conversion
    double mn, mx;
    cv::minMaxLoc( img, &mn, &mx);
    cv::Mat iimg;   // Convert to CV_8U for Canny
    img.convertTo( iimg, CV_8U, 255./mx);

    // Take the edge image using provided parameters
    cv::Mat cimg;
    cv::Canny( iimg, cimg, lowCT, highCT);
    cimg.convertTo( cimg, CV_8U, -1, 255);  // Invert produced edge map

    //const cv::Mat dimg = RFeatures::convertForDisplay( cimg, true);
    //RFeatures::showImage( dimg, "Canny edges", false);
    return cimg;
}   // end createBinaryEdgeMap



// protected
FeatureExtractor::Ptr EDTFeatureExtractor::createFromParams( const string& params) const
{
    int lowCT, highCT;
    cv::Size fvDims;
    try
    {
        std::istringstream iss(params);
        iss >> lowCT >> highCT >> fvDims.width;
        fvDims.height = fvDims.width;
    }   // end try
    catch ( const std::exception&)
    {
        throw ExtractorTypeException( "Couldn't read EDTFeatureExtractor params from string: " + params);
    }   // end catch

    EDTFeatureExtractor* fx = new EDTFeatureExtractor;
    fx->_lowCT = lowCT;
    fx->_highCT = highCT;
    fx->_fvDims = fvDims;
    return FeatureExtractor::Ptr(fx);
}   // end createFromParams



// protected
FeatureExtractor::Ptr EDTFeatureExtractor::initExtractor( const cv::Mat img) const
{
    return FeatureExtractor::Ptr( new EDTFeatureExtractor( _lowCT, _highCT, _fvDims, img));
}   // end initExtractor



cv::Mat_<float> EDTFeatureExtractor::extractFV( const cv::Rect rct) const
{
    assert( _edt != NULL);
    const cv::Mat_<float> fv = (*_edt)( rct);
    return RFeatures::toRowVector(fv);
}   // end extractFV
