/************************************************************************
 * Copyright (C) 2017 Richard Palmer
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 ************************************************************************/

/**
 * Segments a binary image into separate 8-connected regions.
 *
 * Richard Palmer
 * November 2014
 */

#ifndef RFEATURES_IMAGE_LABELLER_H
#define RFEATURES_IMAGE_LABELLER_H

#ifdef _WIN32
#pragma warning( disable : 4251)
#endif

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
