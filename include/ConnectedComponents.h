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
#ifndef RFEATURES_CONNECTED_COMPONENTS_H
#define RFEATURES_CONNECTED_COMPONENTS_H

#include "rFeatures_Export.h"
#include <opencv2/opencv.hpp>
#include <vector>
using std::vector;

typedef vector<cv::Point> Contour;
typedef vector<Contour> ContoursVector;


namespace RFeatures
{

class rFeatures_EXPORT ConnectedComponents
{
public:
    ConnectedComponents( const cv::Mat &img, int minContourLen, int maxContourLen=-1);
    virtual ~ConnectedComponents(){}

    void setContourLengths( int minLen, int maxLen);

    // Returns the connected components from the image having minLen <= lengths < maxLen
    ContoursVector findComponents() const;

    // Filter the provided set of components according to this objects set contour lengths.
    ContoursVector filterComponents( const ContoursVector &cv) const;

    // For all the following drawing functions, -1 may be given to thick to cause the contour
    // to be filled. Positive values denote boundary thickness.

    // Draw the provided contours on the given image with line colour and thickness.
    static void drawContours( const ContoursVector &cv, cv::Mat &img,
                              cv::Scalar colour=cv::Scalar(255,255,255), int thick=1);

    // Draw the convex hulls of the provided contours on the given image.
    static void drawConvexHulls( const ContoursVector &cv, cv::Mat &img,
                                 cv::Scalar colour=cv::Scalar(255,255,255), int thick=1);

    // Draw the approximately fitting polygons of the provided contours on the given image.
    static void drawApproxPolys( const ContoursVector &cv, cv::Mat &img,
                                 cv::Scalar colour=cv::Scalar(255,255,255), int thick=1);

private:
    cv::Mat image;
    size_t minLen;
    size_t maxLen;
};  // end class ConnectedComponents

}   // end namespace RFeatures

#endif
