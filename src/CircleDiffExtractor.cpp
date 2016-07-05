#include "CircleDiffExtractor.h"
using RFeatures::CircleDiffExtractor;
#include <sstream>
#include <cassert>

CircleDiffExtractor::CircleDiffExtractor( int nps, cv::Mat m)
    : FeatureExtractor(m.size()), _nps(nps), _cdiff( new CircleDiff( m, nps))
{}   // end ctor


CircleDiffExtractor::CircleDiffExtractor()
    : FeatureExtractor(), _nps(8), _cdiff(NULL)
{}   // end ctor


CircleDiffExtractor::~CircleDiffExtractor()
{
    if ( _cdiff != NULL)
        delete _cdiff;
}   // end dtor



void CircleDiffExtractor::getValidImageTypes( vector<ImageType>& vimgTypes) const
{
    vimgTypes.push_back(Depth);
    vimgTypes.push_back(Light);
}   // end getValidImageTypes



string CircleDiffExtractor::getParams() const
{
    std::ostringstream oss;
    oss << _nps;
    return oss.str();
}   // end getParams



FeatureExtractor::Ptr CircleDiffExtractor::createFromParams( const string& params) const
{
    int nps;
    try
    {
        std::istringstream iss(params);
        iss >> nps;
    }   // end try
    catch ( const std::exception&)
    {
        throw ExtractorTypeException( "Couldn't read params for CircleDiffExtractor from string: " + params);
    }   // end catch

    CircleDiffExtractor* fx = new CircleDiffExtractor;
    fx->_nps = nps;
    return FeatureExtractor::Ptr(fx);
}   // end createFromParams



FeatureExtractor::Ptr CircleDiffExtractor::initExtractor( const cv::Mat img) const
{
    return FeatureExtractor::Ptr( new CircleDiffExtractor( _nps, img));
}   // end initExtractor



cv::Mat_<float> CircleDiffExtractor::extractFV( const cv::Rect rct) const
{
    assert( _cdiff != NULL);
    const cv::Mat fv = (*_cdiff)( rct);
    return RFeatures::toRowVector(fv);
}   // end extractFV
