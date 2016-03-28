#include "ViewBuilder.h"
using RFeatures::ViewBuilder;
#include <cassert>
#include <iostream>
using std::cerr;
using std::endl;


ViewBuilder::ViewBuilder( int width, int height)
{
    reset( width, height);
}   // end ctor



ViewBuilder::Ptr ViewBuilder::create( int width, int height)
{
    return ViewBuilder::Ptr( new ViewBuilder( width, height));
}   // end create



void ViewBuilder::setPosition( const cv::Vec3d &p)
{
    view_->posVec = p;
}   // end setPosition



void ViewBuilder::setFocalVector( const cv::Vec3d &fv)
{
    view_->focalVec = fv / cv::norm(fv);  // Ensure normalised
}   // end setFocalVector



void ViewBuilder::setUpVector( const cv::Vec3d &uv)
{
    view_->upVec = uv / cv::norm(uv);  // Ensure normalised
}   // end setUpVector



void ViewBuilder::setImage( const cv::Mat_<cv::Vec3b> &img)
{
    view_->img2d = img;
}   // end setImage



View::Ptr ViewBuilder::setPoint( int row, int col, double x, double y, double z, double rng, byte r, byte g, byte b)
{
    cv::Vec3b &pxl = view_->img2d(row,col);    // Colour pixel
    pxl[2] = r;
    pxl[1] = g;
    pxl[0] = b;

    // Pixel is black in point cloud if point is invalid
    if ( x == 0 && y == 0 && z == 0)
    {
        r = 0;
        g = 0;
        b = 0;
    }   // end if

    cv::Vec3f &xyz = view_->points( row, col);
    xyz[0] = x;
    xyz[1] = y;
    xyz[2] = z;
    view_->rngImg(row, col) = rng;  // Range value 

    rngBuilder_->setPointRange( row, col, rng);

    int depthTrue = 0;
    if ( rng > 0)
        depthTrue = 1;
    rngCntImage_->addValue( &depthTrue);

    pointsSet_++;

    const bool ready = isViewReady();
    return ready ? view_ : View::Ptr();
}   // end setPoint



bool ViewBuilder::isViewReady() const
{
    return getProportionReady() == 1.0;
}   // end isViewReady



float ViewBuilder::getProportionReady() const
{
    return (float)pointsSet_ / view_->points.total();
}   // end getProportionReady



void ViewBuilder::setPointPos( int row, int col, double x, double y, double z)
{
    row1_ = row;
    col1_ = col;
    x_ = x;
    x_ = y;
    z_ = z;
    testSetPoint();
}   // end setPointRange



void ViewBuilder::setPointCol( int row, int col, byte r, byte g, byte b)
{
    row2_ = row;
    col2_ = col;
    r_ = r;
    g_ = g;
    b_ = b;
    testSetPoint();
}   // end setPointCol



void ViewBuilder::setPointRange( int row, int col, double rng)
{
    row3_ = row;
    col3_ = col;
    rng_ = rng;
    testSetPoint();
}   // end setPointRange



void ViewBuilder::testSetPoint()
{
    assert( row1_ == row2_ && row2_ == row3_ && col1_ == col2_ && col2_ == col3_);
    setPoint( row1_, col1_, x_, x_, z_, rng_, r_, g_, b_);
}   // end testSetPoint



void ViewBuilder::reset( int width, int height)
{
    pointsSet_ = 0;
    view_ = View::create( width, height);
    rngBuilder_ = RangeBuilder::create( width, height);
    rngCntImage_ = IntegralImage<int>::create( cv::Size(width,height));
}   // end reset
