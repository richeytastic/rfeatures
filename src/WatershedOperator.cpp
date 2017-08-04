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

#include "WatershedOperator.h"
using RFeatures::WatershedOperator;


WatershedOperator::WatershedOperator( const cv::Mat &img, const cv::Mat &mkrs)
{
    image = img.clone();
    mkrs.convertTo( markers, CV_32S);   // Convert to image of ints
}   // end ctor



cv::Mat WatershedOperator::findSegmentation() const
{
    cv::Mat mkrs = markers.clone();    // Image area markers (ints)
    cv::watershed( image, mkrs);
    return mkrs;
}   // end findSegmentation



// static
cv::Mat WatershedOperator::getSegmentedImage( const cv::Mat &mkrs)
{
    cv::Mat tmp;
    // All segments with labels higher than 255 will be assigned value 255
    mkrs.convertTo( tmp, CV_8U);
    return tmp;
}   // end getSegmentedImage



// static
cv::Mat WatershedOperator::getWatershedImage( const cv::Mat &mkrs)
{
    cv::Mat tmp;
    // Each pixel p is transformed into 255p+255 before conversion
    mkrs.convertTo( tmp, CV_8U, 255, 255);
    return tmp;
}   // end getWatershedImage
