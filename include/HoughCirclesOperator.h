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

#pragma once
#ifndef RFEATURES_HOUGH_CIRCLES_OPERATOR_H
#define RFEATURES_HOUGH_CIRCLES_OPERATOR_H

#include "rFeatures_Export.h"
#include "RFeatures.h"

namespace RFeatures
{
class rFeatures_EXPORT HoughCirclesOperator
{
public:
    // Image should be Gaussian blurred before being input to this operator!
    // Eg: cv::GaussianBlur( image, image, cv::Size(5,5), 1.5)
    HoughCirclesOperator( const cv::Mat &image, int highCannyThresh=100, int minVotes=100,
                        int minDistance=0, int minRadius=0, int maxRadius=0);

    virtual ~HoughCirclesOperator(){}

    // Configuration parameters
    void setDetectionParams( int highCannyThresh, int minVotes);
    void setCircleParams( int minDistance, int minRadius, int maxRadius);

    // Detect circles where the components of the vector are the X,Y coords
    // of the centre of the circle and the radius.
    void findCircles( Circles &circles) const;

    // Draw given circles on given image in given colour (default white) and thickness
    static void drawCircles( const Circles &circles, cv::Mat &img,
            cv::Scalar colour=cv::Scalar(255,255,255), int thick=1);

private:
    cv::Mat image;      // The image that we're detecting circles on
    int highCannyThresh;// Canny edge detector high threshold
    int minVotes;       // Min accumulator votes for a circle to be identified
    int minDistance;    // Min distance in pixels between circle centres
    int minRadius;      // Min radius of a detected circle
    int maxRadius;      // Max radius of a detected circle

    static const int ACC_RES;   // Inverse ratio of accumulator resolution to image resolution
};  // end class HoughCirclesOperator

}   // end namespace RFeatures

#endif
