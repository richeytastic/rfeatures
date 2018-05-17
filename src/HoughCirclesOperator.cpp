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

#include <HoughCirclesOperator.h>
using RFeatures::HoughCirclesOperator;


const int HoughCirclesOperator::ACC_RES = 1;


HoughCirclesOperator::HoughCirclesOperator( const cv::Mat &img, int hct, int mv, int md, int minR, int maxR)
{
    if ( img.channels() != 1)
        cv::cvtColor( img, image, CV_BGR2GRAY);
    else
        image = img.clone();

    setDetectionParams( hct, mv);
    setCircleParams( md, minR, maxR);
}   // end ctor



void HoughCirclesOperator::setDetectionParams( int hct, int mv)
{
    highCannyThresh = hct;
    minVotes = mv;
}   // end setDetectionParams



void HoughCirclesOperator::setCircleParams( int md, int minR, int maxR)
{
    minDistance = md;
    minRadius = minR;
    maxRadius = maxR;
}   // end setCircleParams



void HoughCirclesOperator::findCircles( Circles &circles) const
{
    cv::HoughCircles( image, circles, CV_HOUGH_GRADIENT,
            HoughCirclesOperator::ACC_RES,
            minDistance, highCannyThresh, minVotes, minRadius, maxRadius);
}   // end findCircles



void HoughCirclesOperator::drawCircles( const Circles &circles, cv::Mat &img, cv::Scalar col, int thick)
{
    Circles::const_iterator it = circles.begin();
    while ( it != circles.end())
    {
        cv::Point p( (int)(*it)[0], (int)(*it)[1]);
        cv::circle( img, p, (int)(*it)[2], col, thick);
        ++it;
    }   // end while
}   // end drawCircles
