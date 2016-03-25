#include "ImageBuilder.h"
using RFeatures::ImageBuilder;



ImageBuilder::Ptr ImageBuilder::create( int width, int height)
{
    return ImageBuilder::Ptr( new ImageBuilder( width, height));
}   // end create



ImageBuilder::ImageBuilder( int width, int height)
{
    reset( width, height);
}   // end ctor



void ImageBuilder::setPointCol( int row, int col, byte r, byte g, byte b)
{
    cv::Vec3b &v = img(row, col);
    v[2] = r; v[1] = g; v[0] = b; // Reverse indices for OpenCV
}   // end setPointCol



void ImageBuilder::reset( int width, int height)
{
    img.create( height, width);
}   // end reset
