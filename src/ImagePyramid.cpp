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

#include "ImagePyramid.h"
using RFeatures::ImagePyramid;
#include <cmath>



ImagePyramid::ImagePyramid( const cv::Mat &img, int octaves, int lambda, int allowedMin)
{
    if ( octaves < 1)
        octaves = 1;
    if ( lambda < 1)
        lambda = 1;
    if ( allowedMin < 1)
        allowedMin = 1;

    // Allowed minimum size can never be greater than the maximum sized image in this pyramid
    int minSize = img.rows;
    if ( img.cols < minSize)
        minSize = img.cols;
    if ( allowedMin > 2*minSize)
        allowedMin = 2*minSize;

    imgStack.reserve(octaves * lambda);
    scales.reserve(octaves * lambda);

    double alpha = pow(2.0,(1.0/lambda)); // Base for scaling
    double scale = 0.0;
    cv::Mat dimg;
    int newRows = 0;
    int newCols = 0;
    for ( int j = 0; j < octaves; ++j)
    {
        for ( int i = 0; i < lambda; ++i)
        {
            scale = pow(alpha, lambda-i) / pow(2,j);
            newRows = cvRound(img.rows * scale);
            newCols = cvRound(img.cols * scale);
            if ( newRows < allowedMin || newCols < allowedMin)
                break;  // We've already reached the minimum allowed size!

            // This is a valid size so create the new image and add to the pyramid
            cv::resize( img, dimg, cv::Size( newCols, newRows));
            imgStack.push_back( dimg);
            scales.push_back( scale);
        }   // end for - interval done
    }   // end for - octave done
}   // end ctor



cv::Mat ImagePyramid::getImage( int idx) const
{
    if ( idx >= (int)imgStack.size())
        return cv::Mat();
    return imgStack[idx];
}   // end getImage



double ImagePyramid::getScale( int idx) const
{
    if ( idx >= (int)imgStack.size())
        return -1;
    return scales[idx];
}   // end getScale
