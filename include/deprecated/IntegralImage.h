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
 * Create an integral image from a multi channel image.
 * The integral image is where every pixel is the sum of itself
 * and those above and to the left. It is useful because it allows for easy
 * computation of the sum of the pixels within any rectangular area.
 */

#pragma once
#ifndef RFEATURES_INTEGRAL_IMAGE_H
#define RFEATURES_INTEGRAL_IMAGE_H
// Disable warnings about MSVC compiler not implementing exception specifications
#ifdef _WIN32
#pragma warning( disable : 4290)
#endif

#include "FeatureExceptions.h"
using RFeatures::ImageOutOfBoundsException;
#include <boost/shared_ptr.hpp>
#include <opencv2/opencv.hpp>
typedef unsigned char byte;
#include <cstdlib>
#include "rFeatures_Export.h"


namespace RFeatures
{

// Templated by the type of the provided cv::Mat in IntegralImage c'tor.
template <typename T>
class rFeatures_EXPORT IntegralImage
{
public:
    typedef boost::shared_ptr<IntegralImage<T> > Ptr;

    static Ptr create( const cv::Size &sz, int channels=1);

    IntegralImage( const cv::Mat &img);
    ~IntegralImage();

    // Create space for an integral image with provided number of channels
    // (points to be provided using addValue).
    IntegralImage( const cv::Size &sz, int channels=1);

    // Add a value to the integral image constructed using c'tor version 2 above.
    // Return true while user should continue adding values. Values are added from
    // from row 0 (at top of image) up (to bottom of image) and from column 0 (from
    // left of image) up (to right of image). Provided parameter is interpretted as
    // and array of values of the same length as the number of channels set
    // in the constructor.
    bool addValue( const T *val);

    // The only rule about creating integral images is that the value to be
    // set at pixel col,row can only be set if values have already been added
    // at pixels col-1,row and col,row-1 UNLESS the value to be set is at the
    // top or left edge of the integral image in which case only the pixel to
    // the left (if on top row) or above (if on left edge) must already have
    // been set. If no pixels have yet been set, the only valid location to
    // set a value is at the origin (top-left) i.e. col=0, row=0.
    // This function allows the user to specify how they wish to fill up
    // the integral image by giving the desired col and row, however the user
    // should be aware of the above constraints in creating the integral image.
    // Only indices matching the above rules will be accepted. True is returned
    // iff the value was set (i.e. the given indices follow the rules for
    // integral image creation). User should not mix calls to addValue and setValue
    // because addValue will always add from the lowest value row on the left and
    // this may not be what is intended (and because this isn't made clear by
    // addValue, user may lose track of the indices and be unable to call
    // setValue again with success).
    bool setValue( int col, int row, const T *val);

    // Return the sum of the pixels in the given rectangle for the given channel or
    // over all channels if constant ALL_CHANNELS is specified. Channel count starts at zero.
    double operator()( const cv::Rect &r, int channel) const throw (ImageOutOfBoundsException);
    double operator()( const cv::Rect &r) const throw (ImageOutOfBoundsException);  // ALL_CHANNELS

    // Set the sum of the pixels in the given rectangle over all channels.
    // The provided array must have length equal to the number of channels in the integral image.
    void operator()( const cv::Rect &r, double *result) const throw (ImageOutOfBoundsException);

    inline cv::Mat getImage() const { return intImg_;}
    inline int channels() const { return intImg_.channels();}
    const int rows, cols;   // Publically accessible members ala cv::Mat
    inline cv::Size size() const { return intImg_.size();}

    static const int ALL_CHANNELS;

private:
    cv::Mat intImg_; // The integral image itself
    int rowIdx_, colIdx_; // Used when constructing an integral image using addValue method
    int* addVec_;   // How many columns (from left to right) have had values added for a row (array index)

    void addToImage( int col, int row, const T*);
    void updateIndices();
};  // end class IntegralImage


#include "template/IntegralImage_template.h"

}   // end namespace RFeatures

#endif
