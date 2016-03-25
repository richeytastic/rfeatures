/**
 * Segments a binary image into separate 8-connected regions.
 *
 * Richard Palmer
 * November 2014
 */

#pragma once
#ifndef RFEATURES_IMAGE_LABELLER_H
#define RFEATURES_IMAGE_LABELLER_H

#include "rFeatures_Export.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include <boost/unordered_map.hpp>
typedef unsigned char byte;


namespace RFeatures
{

class rFeatures_EXPORT ImageLabeller
{
public:
    // Does not segment until operator() called.
    ImageLabeller( const cv::Mat_<byte>& binaryImg, byte fgVal=255);

    // Segment the image into the foreground regions returning the number of regions.
    // If no foreground regions are found (e.g. for a black image), 0 is returned.
    int operator()();

    int getNumRegions() const;  // Returns number of regions (may be zero).

    // Return the points making up the specified region. Region labels
    // are numbered from 0 to getNumRegions() exclusive.
    // Returns NULL if fgRegion is out of range.
    const std::vector<cv::Point>* getRegionPoints( int fgRegion) const;

    // Set the size of each region (number of pixels) in provided vector.
    // Returns the number of regions (entries in regSizes on return).
    // (NB provided vector is initially cleared).
    int getRegionSizes( std::vector<int>& regSizes) const;

    // Convenience function to return the region having the largest
    // number of points. Returns NULL if no regions exist.
    const std::vector<cv::Point>* getLargestRegion() const;

    // DEBUG
    const cv::Mat_<byte> getLabelImage() const { return _labImg;}

private:
    const cv::Mat_<byte> _bimg;
    const byte _fgVal;
    std::vector< std::vector<cv::Point> > _regions;
    cv::Mat_<byte> _labImg; // Label image for display *DEBUG*
};  // end class

}   // end namespace

#endif
