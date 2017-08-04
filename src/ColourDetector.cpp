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

#include <ColourDetector.h>
using namespace RFeatures;


ColourDetector::ColourDetector() : m_minDist(30), m_target(0,0,0)
{
}   // end ctor


ColourDetector::ColourDetector( float minDist, uchar r, uchar g, uchar b)
{
    setColourDistanceThreshold( minDist);
    setTargetColour( r, g, b);
}   // end ctor


ColourDetector::ColourDetector( float minDist, const cv::Vec3b &target)
{
    setColourDistanceThreshold( minDist);
    setTargetColour( target);
}   // end ctor



void ColourDetector::setColourDistanceThreshold( float minDist)
{
    if (minDist < 0.0)
        minDist = 0.0;
    m_minDist = minDist;
}   // end setColourDistanceThreshold



void ColourDetector::setTargetColour( uchar r, uchar g, uchar b)
{
    // Temporary 1-pixel image
    cv::Mat tmp(1,1,CV_8UC3);
    // Note BGR order!
    tmp.at<cv::Vec3b>(0,0)[0] = b;
    tmp.at<cv::Vec3b>(0,0)[1] = g;
    tmp.at<cv::Vec3b>(0,0)[2] = r;
    // Convert to Lab colour space
    cv::cvtColor(tmp,tmp,CV_BGR2Lab);
    m_target = tmp.at<cv::Vec3b>(0,0);
}   // end setTargetColour



void ColourDetector::setTargetColour( const cv::Vec3b &col)
{
    setTargetColour( col[2], col[1], col[0]);
}   // end setTargetColour



cv::Mat ColourDetector::process( const cv::Mat &img)
{
    // Re-allocate binary map if necessary
    m_result.create( img.rows, img.cols, CV_8U);

    // Re-allocate intermediate image if necessary
    m_converted.create( img.rows, img.cols, img.type());

    // Convert to Lab colour space
    cv::cvtColor( img, m_converted, CV_BGR2Lab);

    cv::Mat_<cv::Vec3b>::const_iterator it = m_converted.begin<cv::Vec3b>();
    cv::Mat_<cv::Vec3b>::const_iterator itend = m_converted.end<cv::Vec3b>();
    cv::Mat_<uchar>::iterator itout = m_result.begin<uchar>();

    for ( ; it != itend; ++it, ++itout)
    {
        if ( getNormDistanceEuclidean( *it) < m_minDist)
            *itout = 255;
        else
            *itout = 0;
    }   // end for

    return m_result;
}   // end process



int ColourDetector::getNormDistanceCityBlock( const cv::Vec3b &col) const
{
    return (int)static_cast<float>(abs(col[0] - m_target[0]) + abs(col[1] - m_target[1]) + abs(col[2] - m_target[2]));
}   // end getNormDistanceCityBlock



int ColourDetector::getNormDistanceEuclidean( const cv::Vec3b &col) const
{
    int d = static_cast<int>(
            cv::norm<int,3>( cv::Vec3i( col[0]-m_target[0], col[1]-m_target[1], col[2]-m_target[2])));
    return d;
}   // end getNormDistanceEuclidean
