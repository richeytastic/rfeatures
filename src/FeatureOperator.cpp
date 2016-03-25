#include "FeatureOperator.h"
using RFeatures::FeatureOperator;
#include <cassert>
#include <iostream>
using std::cerr;
using std::endl;



FeatureOperator::FeatureOperator( const cv::Size& sd, const cv::Size fvDims) throw (FeatureSizeException)
    : _srcDims(0,0,sd.width,sd.height), _fvDims(fvDims)
{
    if ( fvDims.height > sd.height || fvDims.width > sd.width)
    {
        cerr << "WARNING (FeatureOperator): sample dims (" << sd.width << ", " << sd.height << ") <= feature (encoding) dims of ("
             << fvDims.width << ", " << fvDims.height << ")" << endl;
        //throw FeatureSizeException("Image size is too small to extract feature vector with specified minimum dimensions!");
    }   // end if
}   // end ctor



cv::Mat_<float> FeatureOperator::operator()() const
{
    cv::Mat_<float> fv;
    if ( _fvDims.area() > 0)
        fv = this->discretelySample( _srcDims, _fvDims);
    else
        fv = this->extract( _srcDims);
    return fv;
}   // end operator()



cv::Mat_<float> FeatureOperator::operator()( const cv::Rect &sr) const 
{
    const cv::Rect intRct = sr & _srcDims;
    cv::Mat_<float> fv;
    if ( _fvDims.area() > 0)
        fv = this->discretelySample( intRct, _fvDims);
    else
        fv = this->extract( intRct);    // Calls child extract if overridden
    return fv;
}   // end operator()



cv::Mat_<float> FeatureOperator::extract( const cv::Rect& rct) const
{
    const cv::Size fvDims = rct.size();
    return discretelySample( rct, fvDims);
}   // end extract



cv::Mat_<float> FeatureOperator::discretelySample( const cv::Rect& rct, const cv::Size& fvDims) const
{
    vector<cv::Mat> simgs;
    getSampleChannels( rct, simgs);
    assert( !simgs.empty());

    const int sdepth = simgs[0].depth();
    const int nrows = simgs[0].rows;
    const int ncols = simgs[0].cols;

#ifndef NDEBUG
    for ( int i = 0; i < simgs.size(); ++i)
    {
        assert( !simgs[i].empty());
        assert( simgs[i].channels() == 1);
        assert( simgs[i].depth() == sdepth);
        assert( simgs[i].rows == nrows);
        assert( simgs[i].cols == ncols);
    }   // end for
#endif

    cv::Mat_<float> fv( 0, nrows*ncols);    // Output fv will have nimgs rows

    const int nimgs = simgs.size();
    for ( int i = 0; i < nimgs; ++i)
    {
        cv::Mat ofv, ffv;
        cv::resize( simgs[i], ofv, fvDims);
        ofv.convertTo( ffv, CV_32F);
        fv.push_back( ffv.reshape( 0, 1));
    }   // end for
    /*
    switch ( sdepth)
    {
        case CV_32F:
            fv = RFeatures::dsample<float>( simgs, fvDims);
            break;
        case CV_64F:
            fv = RFeatures::dsample<double>( simgs, fvDims);
            break;
        case CV_32S:
            fv = RFeatures::dsample<int32_t>( simgs, fvDims);
            break;
        case CV_16S:
            fv = RFeatures::dsample<int16_t>( simgs, fvDims);
            break;
        case CV_16U:
            fv = RFeatures::dsample<uint16_t>( simgs, fvDims);
            break;
        case CV_8S:
            fv = RFeatures::dsample<char>( simgs, fvDims);
            break;
        case CV_8U:
            fv = RFeatures::dsample<unsigned char>( simgs, fvDims);
            break;
        default:
            assert(false);
            break;
    }   // end switch
    */

    return fv;
}   // end discretelySample



void FeatureOperator::getSampleChannels( const cv::Rect& rct, vector<cv::Mat>& simgs) const
{
    throw RFeatures::FeatureException( "FeatureOperator: must override getSamplingImages() if not using default extraction");
}   // end getSampleChannels
