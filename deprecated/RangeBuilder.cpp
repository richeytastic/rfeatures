#include "RangeBuilder.h"
using RFeatures::RangeBuilder;
#include <cassert>
#include <sys/types.h>


const float RangeBuilder::MAX_ALLOWED_RANGE = FLT_MAX;



RangeBuilder::RangeBuilder( int width, int height)
{
    reset( width, height);
}   // end ctor



RangeBuilder::Ptr RangeBuilder::create( int width, int height)
{
    return Ptr( new RangeBuilder( width, height));
}   // end create



void RangeBuilder::setPointRange( int row, int col, double rng)
{
    if ( rng < 0) rng = 0;
    if ( rng > MAX_ALLOWED_RANGE) rng = MAX_ALLOWED_RANGE;
    float frng = (float)rng;    // Always in range of a float
    rngData_(row,col) = frng;
    if ( frng > maxRange_) maxRange_ = frng;
    rngIntImg_->setValue( col, row, &rng);
}   // end setPointRange



void RangeBuilder::reset( int width, int height)
{
    rngData_.create( width, height);
    rngIntImg_ = IntegralImage<double>::create( cv::Size(width,height));
    maxRange_ = 0;
}   // end reset
