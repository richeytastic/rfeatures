#include "SobelEdgesExtractor.h"
using RFeatures::SobelEdgesExtractor;
#include <cassert>
#include <sstream>


SobelEdgesExtractor::SobelEdgesExtractor( int d, cv::Size fvd, cv::Mat img)
    : FeatureExtractor(img.size()), _deriv(d), _fvDims(fvd), _sobelx( new SobelEdges( img, d, 0, fvd)), _sobely( new SobelEdges( img, 0, d, fvd))
{}   // end ctor


SobelEdgesExtractor::SobelEdgesExtractor()
    : FeatureExtractor(), _deriv(0), _fvDims(0,0), _sobelx( NULL), _sobely( NULL)
{}   // end ctor


SobelEdgesExtractor::~SobelEdgesExtractor()
{
    if ( _sobelx != NULL)
    {
        delete _sobelx;
        delete _sobely;
    }   // end if
}   // end dtor


void SobelEdgesExtractor::getValidImageTypes( vector<ImageType>& vimgTypes) const
{
    vimgTypes.push_back(Depth); 
    vimgTypes.push_back(Grey); 
    vimgTypes.push_back(Light);
}   // end getValidImageTypes


string SobelEdgesExtractor::getParams() const
{
    std::ostringstream ss;
    ss << _deriv << " " << _fvDims.width;
    return ss.str();
}   // end getParams



FeatureExtractor::Ptr SobelEdgesExtractor::createFromParams( const string& params) const
{
    int deriv;
    cv::Size fvDims;
    try
    {
        std::istringstream iss(params);
        iss >> deriv >> fvDims.width;
        fvDims.height = fvDims.width;
    }   // end try
    catch ( const std::exception& e)
    {
        throw ExtractorTypeException( "Couldn't read SobelEdgesExtractor params from string: " + params);
    }   // end catch

    SobelEdgesExtractor* fx = new SobelEdgesExtractor;
    fx->_deriv = deriv;
    fx->_fvDims = fvDims;
    return FeatureExtractor::Ptr(fx);
}   // end createFromParams



FeatureExtractor::Ptr SobelEdgesExtractor::initExtractor( const cv::Mat img) const
{
    return FeatureExtractor::Ptr( new SobelEdgesExtractor( _deriv, _fvDims, img));
}   // end initExtractor



cv::Mat_<float> SobelEdgesExtractor::extractFV( const cv::Rect rct) const
{
    assert( _sobelx != NULL);
    assert( _sobely != NULL);
    const cv::Mat fvx = (*_sobelx)( rct);
    const cv::Mat fvy = (*_sobely)( rct);
    cv::Mat_<float> fv = RFeatures::toRowVector(fvx);
    cv::Mat_<float> fv1 = RFeatures::toRowVector(fvy);
    assert( fv1.cols == fv.cols);
    fv.push_back(fv1);
    return fv.reshape(0,1); // Single row vector
}   // end extractFV
