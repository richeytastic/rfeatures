#include "FastHOGExtractor.h"
using RFeatures::FastHOGExtractor;
#include <sstream>
#include <cassert>


FastHOGExtractor::FastHOGExtractor( int nbins, cv::Size pw, cv::Size fd, cv::Mat img)
    : FeatureExtractor(img.size()), _nbins(nbins), _pxlWin(pw), _fvDims(fd), _fhog( new FastHOG( img, nbins, pw, fd))
{}   // end ctor


FastHOGExtractor::FastHOGExtractor()
    : FeatureExtractor(), _nbins(0), _pxlWin(0,0), _fvDims(0,0), _fhog(NULL)
{}   // end ctor


FastHOGExtractor::~FastHOGExtractor()
{
    if ( _fhog != NULL)
        delete _fhog;
}   // end dtor


void FastHOGExtractor::getValidImageTypes( vector<ImageType>& vimgTypes) const
{
    vimgTypes.push_back(BGR);
    vimgTypes.push_back(Grey);
    vimgTypes.push_back(Light);
    vimgTypes.push_back(CIELab);
    vimgTypes.push_back(Depth);
    vimgTypes.push_back(EDT);
}   // end getValidImageTypes



string FastHOGExtractor::getParams() const
{
    std::ostringstream oss;
    oss << _nbins << " " << _pxlWin.width << " " << _fvDims.width;
    return oss.str();
}   // end getParams


// protected
FeatureExtractor::Ptr FastHOGExtractor::createFromParams( const string& params) const
{
    int nbins;
    cv::Size pxlWin, fvDims;
    try
    {
        std::istringstream iss(params);
        iss >> nbins >> pxlWin.width >> fvDims.width;
        pxlWin.height = pxlWin.width;
        fvDims.height = fvDims.width;
    }   // end try
    catch ( const std::exception&)
    {
        throw ExtractorTypeException( "Unable to parse params for FastHOGExtractor: " + params);
    }   // end catch

    FastHOGExtractor* fx = new FastHOGExtractor;
    fx->_nbins = nbins;
    fx->_pxlWin = pxlWin;
    fx->_fvDims = fvDims;
    return FeatureExtractor::Ptr( fx);
}   // end createFromParams



// protected
FeatureExtractor::Ptr FastHOGExtractor::initExtractor( const cv::Mat img) const
{
    return FeatureExtractor::Ptr( new FastHOGExtractor( _nbins, _pxlWin, _fvDims, img));
}   // end initExtractor



cv::Mat_<float> FastHOGExtractor::extractFV( const cv::Rect rct) const
{
    assert( _fhog != NULL);
    const cv::Mat_<float> fv = (*_fhog)(rct);
    return RFeatures::toRowVector(fv);
}   // end extractFV
