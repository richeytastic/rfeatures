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

#include <Histogram1D.h>
using namespace RFeatures;


Histogram1D::Histogram1D()
{
    m_histSize[0] = 256;
    m_hranges[0] = 0.0;
    m_hranges[1] = 255.0;
    m_ranges[0] = m_hranges;
    m_channels[0] = 0;   // Channel 0 by default
}   // end ctor



cv::MatND Histogram1D::getHistogram( const cv::Mat &img)
{
    cv::MatND hist;

    // Compute histogram
    cv::calcHist( &img,
            1,  // Histogram from 1 image only
            m_channels, // The channel used
            cv::Mat(),  // No mask is used
            hist,       // The resulting histogram
            1,          // Which is 1 dimensional
            m_histSize, // Number of bins
            m_ranges    // Pixel value ranges
            );

    return hist;
}   // end getHistogram



cv::Mat Histogram1D::getHistogramImage( const cv::Mat &img)
{
    cv::MatND hist = getHistogram(img);

    // Get min and max bin values
    double maxVal = 0;
    double minVal = 0;
    cv::minMaxLoc( hist, &minVal, &maxVal, 0, 0);

    // Image on which to display histogram
    cv::Mat histImg( m_histSize[0], m_histSize[0], CV_8U, cv::Scalar(255));

    // Set highest point at 90% of nbins
    int hpt = static_cast<int>(0.9 * m_histSize[0]);

    // Draw a vertical line for each bin
    for ( int k = 0; k < m_histSize[0]; ++k)
    {
        float binVal = hist.at<float>(k);
        int intensity = static_cast<int>( binVal * hpt / maxVal);

        // Draw a line between two points
        cv::line( histImg, cv::Point( k, m_histSize[0]),
                           cv::Point( k, m_histSize[0] - intensity),
                           cv::Scalar::all(0));
    }   // end for

    return histImg;
}   // end getHistogramImage



cv::Mat Histogram1D::applyLookUp( const cv::Mat &img, const cv::Mat &lookup)
{
    cv::Mat result;
    cv::LUT( img, lookup, result);
    return result;
}   // end applyLookUp



cv::Mat Histogram1D::stretch( const cv::Mat &img, int minVal)
{
    Histogram1D h;
    cv::MatND hist = h.getHistogram(img);

    // Find left extremity of the histogram
    int imin = 0;
    for ( ; imin < h.m_histSize[0]; ++imin)
    {
        if ( hist.at<float>(imin) > minVal)
            break;
    }   // end for

    // Find right extremity of histogram
    int imax = h.m_histSize[0] - 1;
    for ( ; imax >= 0; imax--)
    {
        if ( hist.at<float>(imax) > minVal)
            break;
    }   // end for

    // Create lookup table
    int dim = 256;
    cv::Mat lookup(1, &dim, CV_8U);
    for ( int i = 0; i < 256; ++i)
    {
        // Stretch between imin and imax
        if ( i < imin)
            lookup.at<uchar>(i) = 0;
        else if ( i > imax)
            lookup.at<uchar>(i) = 255;
        else    // Linear mapping
            lookup.at<uchar>(i) = static_cast<uchar>( 255.0 * (i-imin)/(imax-imin) + 0.5);
    }   // end for

    // Apply lookup table
    return Histogram1D::applyLookUp(img, lookup);
}   // end stretch



cv::Mat Histogram1D::equalise( const cv::Mat &img)
{
    cv::Mat result;
    cv::equalizeHist( img, result);
    return result;
}   // end equalise
