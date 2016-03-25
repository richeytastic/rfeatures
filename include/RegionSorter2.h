#pragma once
#ifndef RFEATURES_REGION_SORTER2_
#define RFEATURES_REGION_SORTER2_

#include <opencv2/opencv.hpp>
typedef unsigned int uint;


namespace RFeatures
{

class RegionSorter2
{
public:
    static RegionSorter2* create( uint dim);    // Square dimensions and only powers of 2

    // Add a rectangular region with given value. Returns the
    // maximum value so far.
    virtual double add( const cv::Rect& rct, double v) = 0;

    // Return the value and region of the highest valued area.
    virtual double findMax( cv::Rect& outRect) const = 0;
};  // end class

}   // end namespace

#endif
