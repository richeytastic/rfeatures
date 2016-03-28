#include "RangeGradientsBuilder.h"
using RFeatures::RangeGradientsBuilder;
#include "RangeBuilder.h"   // For RangeBuilder::MAX_ALLOWED_RANGE


// static
RangeGradientsBuilder::Ptr RangeGradientsBuilder::create( const cv::Mat_<double> &img, int nbs, bool dd, bool ss)
{
    return RangeGradientsBuilder::Ptr( new RangeGradientsBuilder( img, nbs, dd, ss));
}   // end create

RangeGradientsBuilder::RangeGradientsBuilder( const cv::Mat_<double> &img, int nbs, bool dd, bool ss)
    : byPoint( false)
{
    grads.create( img.size(), nbs, dd, ss);
    const cv::Mat_<cv::Vec<double,1> > &im = (const cv::Mat_<cv::Vec<double,1> >&)img;
    for ( int i = 0; i < img.rows; ++i)
        for ( int j = 0; j < img.cols; ++j)
            grads.addValue( im(i,j));
}   // end ctor


// static
RangeGradientsBuilder::Ptr RangeGradientsBuilder::create( const cv::Mat_<float> &img, int nbs, bool dd, bool ss)
{
    return RangeGradientsBuilder::Ptr( new RangeGradientsBuilder( img, nbs, dd, ss));
}   // end create

RangeGradientsBuilder::RangeGradientsBuilder( const cv::Mat_<float> &img, int nbs, bool dd, bool ss)
    : byPoint( false)
{
    grads.create( img.size(), nbs, dd, ss);
    const cv::Mat_<cv::Vec<float,1> > &im = (const cv::Mat_<cv::Vec<float,1> >&)img;
    for ( int i = 0; i < img.rows; ++i)
        for ( int j = 0; j < img.cols; ++j)
            grads.addValue( im(i,j));
}   // end ctor


// static
RangeGradientsBuilder::Ptr RangeGradientsBuilder::create( const cv::Mat_<byte> &img, int nbs, bool dd, bool ss)
{
    return RangeGradientsBuilder::Ptr( new RangeGradientsBuilder( img, nbs, dd, ss));
}   // end create

RangeGradientsBuilder::RangeGradientsBuilder( const cv::Mat_<byte> &img, int nbs, bool dd, bool ss)
    : byPoint( false)
{
    grads.create( img.size(), nbs, dd, ss);
    const cv::Mat_<cv::Vec<byte,1> > &im = (const cv::Mat_<cv::Vec<byte,1> >&)img;
    for ( int i = 0; i < img.rows; ++i)
        for ( int j = 0; j < img.cols; ++j)
            grads.addValue( im(i,j));
}   // end ctor



// static
RangeGradientsBuilder::Ptr RangeGradientsBuilder::create( int nbs, bool dd, bool ss)
{
    return RangeGradientsBuilder::Ptr( new RangeGradientsBuilder( nbs, dd, ss));
}   // end create



RangeGradientsBuilder::RangeGradientsBuilder( int nbs, bool dd, bool ss)
    : byPoint( true), nbins(nbs), dirDep(dd), spatialSmooth(ss)
{}   // end ctor



void RangeGradientsBuilder::reset( int width, int height)
{
    if ( !byPoint) return;
    cv::Size sz( width, height);
    grads.create( sz, nbins, dirDep, spatialSmooth);
}   // end reset



void RangeGradientsBuilder::setPointRange( int row, int col, double rng)
{
    if ( !byPoint) return;
    if ( rng < 0) rng = 0;
    if ( rng > RangeBuilder::MAX_ALLOWED_RANGE) rng = RangeBuilder::MAX_ALLOWED_RANGE;
    grads.addValue( (const cv::Vec<double,1>&)rng);  // Add gradient
}   // end setPointRange
