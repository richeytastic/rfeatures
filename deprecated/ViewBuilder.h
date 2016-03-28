/**
 * Build a View object incrementally.
 * 
 * Richard Palmer
 * September 2012
 */

#pragma once
#ifndef RFEATURES_VIEW_BUILDER
#define RFEATURES_VIEW_BUILDER

#include <opencv2/opencv.hpp>
#include "PointDataBuilder.h"
using RFeatures::PointDataBuilder;
#include "View.h"
using RFeatures::View;
#include "RangeBuilder.h"
using RFeatures::RangeBuilder;
#include "IntegralImage.h"
using RFeatures::IntegralImage;
#include "DepthFinder.h"
using RFeatures::DepthFinder;


namespace RFeatures
{

class rFeatures_EXPORT ViewBuilder : public PointDataBuilder
{
public:
    typedef boost::shared_ptr<ViewBuilder> Ptr;

    static Ptr create( int width, int height);
    ViewBuilder( int width, int height);
    virtual ~ViewBuilder(){}

    // These functions may be called any time before or after construction is complete.
    void setPosition( const cv::Vec3d&);
    void setFocalVector( const cv::Vec3d&);
    void setUpVector( const cv::Vec3d&);

    void setImage( const cv::Mat_<cv::Vec3b>&);

    bool isViewReady() const;    // True only if all points have been added
    float getProportionReady() const; // Returns number between 0 and 1. Points all set when == 1.

    virtual void setPointPos( int row, int col, double x, double y, double z);
    virtual void setPointCol( int row, int col, byte r, byte g, byte b);
    virtual void setPointRange( int row, int col, double rng);
    // Sets the point at row,col. Returns the view iff all points have been set, otherwise returns NULL.
    View::Ptr setPoint( int row, int col, double x, double y, double z, double rng, byte r, byte g, byte b);

    virtual void reset( int width, int height);

    inline View::Ptr getView() const { return view_;}
    inline DepthFinder::Ptr getDepthFinder() const
    {
        return DepthFinder::create( rngBuilder_->getIntegralRangeImage(), rngCntImage_);
    }   // end getDepthFinder

private:
    View::Ptr view_;
    bool calledSetPos_;
    bool calledSetCol_;
    bool calledSetRng_;
    int pointsSet_;
   
    // Track calls to setPointPos and setPointCol to match up pixel index row,col pairs 
    int row1_, row2_, row3_;  // setPointPos sets m_row1; setPointCol sets m_row2; setPointRange sets m_row3
    int col1_, col2_, col3_;  // setPointPos sets m_col1; setPointCol sets m_col2; setPointRange sets m_col3
    double x_, y_, z_;   // The x,y,z position saved from setPointPos
    byte r_, g_, b_;     // The colour values saved from setPointCol
    double rng_;           // The range value saved from setPointRange

    RangeBuilder::Ptr rngBuilder_;
    IntegralImage<int>::Ptr rngCntImage_;  // Counts range values > 0 only for DepthFinder

    void testSetPoint();
};  // end class

}   // end namespace

#endif
