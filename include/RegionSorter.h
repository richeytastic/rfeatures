/**
 * Finds an ordering of region maxima in a "response map" generated from the
 * sequential addition of rectanglular areas of different dimensions and
 * positions in the map having different values. The response map is an
 * aggregation of these values over their respective rectangular bounding
 * regions. Finding the maximum region takes O(log N) time (where N is
 * the (square) dimension of the response map). Normally, finding the
 * maximum value takes O(N^2) time.
 *
 * Richard Palmer
 * November 2012
 */

#pragma once
#ifndef RFEATURES_REGION_SORTER_
#define RFEATURES_REGION_SORTER_

#include <list>
using std::list;
#include <boost/shared_ptr.hpp>
#include <opencv2/opencv.hpp>
typedef unsigned int uint;
#include "rFeatures_Export.h"


namespace RFeatures
{

class rFeatures_EXPORT RegionSorter
{
public:
    explicit RegionSorter( uint dim); // Square dimensions - only powers of 2 allowed
    virtual ~RegionSorter();

    // Add a rectangular region with given value. Returns the min
    // and max values so far in a 2 element internal.
    virtual double* add( const cv::Rect&, double);

    // Return the rectangular regions of areas having the highest (same) value.
    // Some of the rectangles may intersect or be adjacent (or the same area), or
    // may be very separate (if more than one "peak" with same value exists).
    virtual double findMax( list<cv::Rect>& regions) const;

    // As findMax for finding the minimum regions.
    virtual double findMin( list<cv::Rect>& regions) const;

    // As findMax, but this function removes the area meaning that
    // the next call to removeMax will return the second highest region.
    // In this manner, consecutive calls to removeMax can give a
    // max to min ordering of subset regions by their value.
    // This function is destructive.
    virtual double removeMax( list<cv::Rect>& regions);

private:
    class RegionSorter_impl;
    boost::shared_ptr<RegionSorter_impl> pimpl_;
};  // end class

}   // end namespace

#endif
