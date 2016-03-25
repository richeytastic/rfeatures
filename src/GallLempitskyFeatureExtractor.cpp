#include "GallLempitskyFeatureExtractor.h"
using RFeatures::GallLempitskyFeatureExtractor;
#include <cassert>

GallLempitskyFeatureExtractor::GallLempitskyFeatureExtractor( cv::Mat m)
    : FeatureExtractor(m.size()), _glf( new GallLempitskyFeature( m))
{}  // end ctor


GallLempitskyFeatureExtractor::GallLempitskyFeatureExtractor()
    : FeatureExtractor(), _glf(NULL)
{}  // end ctor


GallLempitskyFeatureExtractor::~GallLempitskyFeatureExtractor()
{
    if ( _glf != NULL)
        delete _glf;
}   // end dtor


string GallLempitskyFeatureExtractor::getParams() const
{
    return "";
}   // end getParams


void GallLempitskyFeatureExtractor::getValidImageTypes( vector<ImageType>& vimgTypes) const
{
    vimgTypes.push_back(BGR);
}   // end getValidImageTypes


FeatureExtractor::Ptr GallLempitskyFeatureExtractor::createFromParams( const string& params) const
{
    return FeatureExtractor::Ptr( new GallLempitskyFeatureExtractor);
}   // end createFromParams


// protected
FeatureExtractor::Ptr GallLempitskyFeatureExtractor::initExtractor( const cv::Mat img) const
{
    return FeatureExtractor::Ptr( new GallLempitskyFeatureExtractor( img));
}   // end initExtractor


cv::Mat_<float> GallLempitskyFeatureExtractor::extractFV( const cv::Rect rct) const
{
    assert( _glf != NULL);
    return (*_glf)( rct);
}   // end extractFV
