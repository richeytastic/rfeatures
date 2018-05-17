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

#include "HoughLinesOperator.h"
using namespace RFeatures;


HoughLinesOperator::HoughLinesOperator( const CannyOperator &co, int minV, int minL, int maxG, int dr, double dt)
    : canny(co)
{
    setDetectionParams( dr, dt, minV);
    setLineParams( minL, maxG);
}   // end ctor



HoughLinesOperator::~HoughLinesOperator()
{}   // end dtor



void HoughLinesOperator::setDetectionParams( int dr, double dt, int minV)
{
    deltaRho = dr;
    deltaTheta = dt;
    if ( minV < 1)
        minV = 1;
    minVote = minV;
}   // end setDetectionParams



void HoughLinesOperator::setLineParams( int minL, int maxG)
{
    if ( minL < 1) minL = 1;
    if ( maxG < 0) maxG = 0;
    minLength = minL;
    maxGap = maxG;
}   // end setLineParams



void HoughLinesOperator::findLines( Lines &lns) const
{
    cv::Mat_<float> contours = canny.getEdgeImage();
    cv::HoughLinesP( contours, lns, deltaRho, deltaTheta, minVote, minLength, maxGap);
}   // end findLines



void HoughLinesOperator::drawLines( const Lines &lines, cv::Mat &img, cv::Scalar col, int thick)
{
    Lines::const_iterator it = lines.begin();
    while ( it != lines.end())
    {
        cv::Point p1((*it)[0], (*it)[1]);
        cv::Point p2((*it)[2], (*it)[3]);
        cv::line( img, p1, p2, col, thick);
        ++it;
    }   // end while
}   // end drawLines
