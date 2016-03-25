/**
 * Lines found in the 2D colour image (e.g. using Hough Transform) are only lines
 * in 3D if the depth along their length varies linearly (within certain bounds).
 *
 * This class takes a vector of 2D lines and sorts them according to the likelihood
 * that they actually represent a line in 3D by comparing the rate of change of depth
 * along their length with the change in depth in the associated linearly scaled range image.
 *
 * Richard Palmer
 * February 2012
 */

#pragma once
#ifndef RFEATURES_LINES_FILTER_H
#define RFEATURES_LINES_FILTER_H

#include "rFeatures_Export.h"
#include <cmath>
#include <opencv2/opencv.hpp>
#include <vector>
typedef std::vector<cv::Vec4i> Lines;


namespace RFeatures
{

class rFeatures_EXPORT LinesFilter
{
public:
    LinesFilter( const Lines &lns, const cv::Mat &rngImg);
    ~LinesFilter();

    // Filters the lines provided in the constructor, returning only those with
    // standard deviation of depth change <= the provided parameter.
    // For efficiency, lines are checked at every approximate stageGap pixels
    // instead of at every pixel covered by the line.
    // To try and avoid edge discontinuities problems (e.g. where the 2D line
    // is incident with an edge discontinuity), the range image is first blurred
    // using a kernel of the given size (filterSz).
    Lines filter( double maxStdDev, int stageGap=2, int filterSz=3);

private:
    Lines lines;
    cv::Mat rngImg;
};  // end class LinesFilter


}   // end namespace RFeatures


#endif



