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

#include <ContentFinder.h>
using namespace RFeatures;


ContentFinder::ContentFinder() : m_threshold(-1.0f)
{
    m_ranges[0] = m_hranges;    // All channels have same range
    m_ranges[1] = m_hranges;
    m_ranges[2] = m_hranges;
}   // end ctor



void ContentFinder::setThreshold( float t)
{
    m_threshold = t;
}   // end setThreshold



void ContentFinder::setHistogram( const cv::MatND &h)
{
    m_histogram = h;
    cv::normalize( m_histogram, m_histogram, 1.0);
}   // end setHistogram



cv::Mat ContentFinder::find( const cv::Mat &img, float minVal, float maxVal, int *channels, int dim)
{
    cv::Mat result;

    m_hranges[0] = minVal;
    m_hranges[1] = maxVal;

    for ( int i = 0; i < dim; ++i)
        m_channels[i] = channels[i];

    cv::calcBackProject( &img, 1, // input image
            channels,   // list of channels used
            m_histogram,    // the histogram we are using
            result,       // the resulting back projection
            m_ranges,   // the range of values
            255.0); // scaling factor

    // Threshold back projection to obtain binary image
    if ( m_threshold > 0.0)
        cv::threshold(result, result, 255 * m_threshold, 255, cv::THRESH_BINARY);

    return result;
}   // end find
