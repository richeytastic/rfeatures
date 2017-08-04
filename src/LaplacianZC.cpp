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

#include <LaplacianZC.h>
using namespace RFeatures;
#include <algorithm>


LaplacianZC::LaplacianZC( int a)
{
    setAperture( a);
}   // end ctor



void LaplacianZC::setAperture( int a)
{
    if ( a < 3)
        a = 3;
    aperture = a;
}   // end setAperture



cv::Mat LaplacianZC::computeLaplacian( const cv::Mat &img)
{
    cv::Mat laplace;
    cv::Laplacian( img, laplace, CV_32F, aperture);
    return laplace;
}   // end computeLaplacian



cv::Mat LaplacianZC::getLaplacianImage( const cv::Mat &img)
{
    cv::Mat laplace = computeLaplacian( img);
    double min, max;
    cv::minMaxLoc( laplace, &min, &max);
    double scale = 127./std::max(-min,max);
    cv::Mat lapImg;
    laplace.convertTo( lapImg, CV_8U, scale, 128);
    return lapImg;
}   // end getLaplacianImage



cv::Mat LaplacianZC::getZeroCrossings( const cv::Mat &img, float threshold)
{
    cv::Mat laplace = computeLaplacian( img);

    cv::Mat_<float>::const_iterator it = laplace.begin<float>() + laplace.step1();
    cv::Mat_<float>::const_iterator itend = laplace.end<float>();
    cv::Mat_<float>::const_iterator itup = laplace.begin<float>();

    // Binary image initialised to white
    cv::Mat binary( laplace.size(), CV_8U, cv::Scalar(255));
    cv::Mat_<uchar>::iterator itout = binary.begin<uchar>() + binary.step1();

    threshold *= -1.0;  // Negate the input threshold value

    for ( ; it != itend; ++it, ++itup, ++itout)
    {
        // Detect zero-crossings:
        // If the product of two adjacent pixels is negative (less than the threshold)
        // then the 2nd order derivative's sign has changed and we have a zero-crossing.
        if (( *it * *(it-1)) < threshold)
            *itout = 0; // Horizontal zero-crossing
        else if (( *it * *itup) < threshold)
            *itout = 0; // Vertical zero-crossing
    }   // end for

    return binary;
}   // end getZeroCrossings
