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

#ifndef RFEATURES_RECT_CLUSTER_H
#define RFEATURES_RECT_CLUSTER_H

#ifdef _WIN32
#pragma warning( disable : 4251)
#endif

#include <opencv2/opencv.hpp>
#include "rFeatures_Export.h"
#include <boost/shared_ptr.hpp>
#include <list>

namespace RFeatures
{

class rFeatures_EXPORT RectCluster
{
public:
    typedef boost::shared_ptr<RectCluster> Ptr;
    static Ptr create( double combAreaProp=0.0);

    // combAreaProp sets the rule for allowing new rectangles to
    // be added to the cluster (see add() below).
    explicit RectCluster( double combAreaProp=0.0);
    ~RectCluster();

    // Returns true IFF r was added to this cluster. Rectangle
    // is only added if it covers either at least C * the current
    // intersection area of this cluster, OR the current intersection
    // area of this cluster covers at least C * the area of the rectangle.
    // (where C = getCombineAreaProportion()).
    bool add( const cv::Rect& r);

    inline const std::list<cv::Rect>& getRectangles() const { return *_rects;}

    // Returns value in (0,1] with 1 being the most compact.
    double calcCompactness() const;

    // Returns value in [0,+inf). Value is normalised by the area containing
    // all of the rectangles in the cluster (the cluster union).
    double calcDensity() const;

    // Calculate how "good" this cluster is. Larger values are better.
    // Three factors increase the quality of the cluster. The clusters
    // density of the rectangles, the compactness of the cluster, and
    // the mean size of the cluster.
    double calcQuality() const;

    inline const cv::Rect& getIntersection() const { return _intersection;}
    inline const cv::Rect& getUnion() const { return _union;}
    inline const cv::Rect_<double>& getMean() const { return _mean;}
    inline double getCombineAreaProportion() const { return _cmbAreaProp;}

    // Returns the area of this cluster as a running total of the rectangles
    // added so far. That is, intersecting areas are counted multiple times.
    double getAggregateArea() const { return _areaSum;}

    // Get the mean centre of the rectangles in the cluster.
    cv::Point getClusterCentre() const;

private:
    std::list<cv::Rect>* _rects;
    double _cmbAreaProp;
    double _areaSum;
    cv::Rect _intersection;
    cv::Rect _union;
    cv::Rect_<double> _mean;
};  // end class


// Given a list of rectangles, create a list of clusters given
// the add restriction of c (see RectCluster::add())
rFeatures_EXPORT void clusterRects( const std::list<cv::Rect>& boxes, double c,
                                    std::vector<RectCluster::Ptr>& clusters);

}   // end namespace

#endif
