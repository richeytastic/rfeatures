#include "DepthDiffExtractor.h"
using RFeatures::PatchPointType;
using RFeatures::DepthDiffExtractor;
#include <algorithm>
#include <sstream>
#include <cassert>


DepthDiffExtractor::DepthDiffExtractor( PatchPointType ppt, float s, cv::Mat m)
    : FeatureExtractor(m.size()), _ppt(ppt), _sensitivity(s), _depthDiff( new DepthDiff(m, ppt, s))
{}   // end ctor



DepthDiffExtractor::DepthDiffExtractor()
    : FeatureExtractor(), _ppt(FOUR_PT), _sensitivity(1), _depthDiff(NULL)
{}   // end ctor


DepthDiffExtractor::~DepthDiffExtractor()
{
    if ( _depthDiff != NULL)
        delete _depthDiff;
}   // end dtor



void DepthDiffExtractor::getValidImageTypes( vector<ImageType>& vimgTypes) const
{
    vimgTypes.push_back(Depth);
    vimgTypes.push_back(EDT);
    vimgTypes.push_back(Light);
}   // end getValidImageTypes



string DepthDiffExtractor::getParams() const
{
    std::ostringstream oss;
    switch (_ppt)
    {
        case FOUR_PT:
            oss << "Four_Pt";
            break;
        case FIVE_PT:
            oss << "Five_Pt";
            break;
        case NINE_PT:
            oss << "Nine_Pt";
            break;
        case THIRTEEN_PT:
            oss << "Thirteen_Pt";
            break;
    }   // end swtich
    return oss.str();
}   // end getParams


PatchPointType fromString( string sppt)
{
    std::transform( sppt.begin(), sppt.end(), sppt.begin(), ::tolower);
    if ( sppt == "four_pt")
        return RFeatures::FOUR_PT;
    else if ( sppt == "five_pt")
        return RFeatures::FIVE_PT;
    else if ( sppt == "nine_pt")
        return RFeatures::NINE_PT;
    else if ( sppt == "thirteen_pt")
        return RFeatures::THIRTEEN_PT;
    else
        throw ExtractorTypeException("Invalid PatchPointType string!");
    return (PatchPointType)-1;
}   // end fromString



FeatureExtractor::Ptr DepthDiffExtractor::createFromParams( const string& params) const
{
    PatchPointType ppt;
    float sensitivity = 1;
    try
    {
        std::istringstream iss(params);
        string sppt;
        iss >> sppt;
        ppt = fromString(sppt);   // May throw
        iss >> sensitivity;
    }   // end try
    catch ( const std::exception&)
    {
        throw ExtractorTypeException( "Cannot read params for DepthDiffExtractor from string: " + params);
    }   // end catch

    DepthDiffExtractor *dd = new DepthDiffExtractor;
    dd->_ppt = ppt;
    dd->_sensitivity = sensitivity;
    return FeatureExtractor::Ptr(dd);
}   // end createFromParams



FeatureExtractor::Ptr DepthDiffExtractor::initExtractor( const cv::Mat img) const
{
    return FeatureExtractor::Ptr( new DepthDiffExtractor( _ppt, _sensitivity, img));
}   // end initExtractor



/*
size_t DepthDiffExtractor::extract( const cv::Rect rct, PatchDescriptor::Ptr pd) const
{
    assert( _depthDiff != NULL);
    const cv::Mat_<float> fv = (*_depthDiff)(rct);
    assert( fv.rows == 1);
    pd->addFeatureVector(fv);
    return fv.rows;
}   // end extract


size_t DepthDiffExtractor::extract( const cv::Mat m, PatchDescriptor::Ptr pd) const
{
    const cv::Mat_<float> fv = DepthDiff( m, _ppt, _sensitivity)();
    pd->addRowFeatureVectors(fv);
    return fv.rows;
}   // end extract
*/


cv::Mat_<float> DepthDiffExtractor::extractFV( const cv::Rect rct) const
{
    assert( _depthDiff != NULL);
    const cv::Mat_<float> fv = (*_depthDiff)(rct);
    assert( fv.rows == 1);
    return RFeatures::toRowVector(fv);
}   // end extractFV
