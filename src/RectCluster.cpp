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

#include "RectCluster.h"
using RFeatures::RectCluster;
#include <boost/foreach.hpp>
#include <cmath>

RectCluster::Ptr RectCluster::create( float cap) { return Ptr( new RectCluster(cap));}
RectCluster::RectCluster( float cap) : _rects( new std::list<cv::Rect>), _cmbAreaProp(cap), _areaSum(0) {}
RectCluster::~RectCluster() { delete _rects;}


bool checkAddRule( const RectCluster& c, const cv::Rect& r)
{
    const cv::Rect& crect = c.getIntersection();
    const float cap = c.getCombineAreaProportion();
    // In order to allow combining of r with c, r must cover
    // at least cap * the current intersection area of c OR
    // cap * r.area() must be covered by the current intersection area.
    const float iarea = (crect & r).area();
    return iarea >= cap * crect.area() || iarea >= cap * r.area();
}   // end checkAddRule


// public
bool RectCluster::add( const cv::Rect& r)
{
    bool added = false;
    if ( _rects->empty())
    {
        _areaSum = r.area();
        _intersection = r;
        _union = r;
        _mean = r;
        added = true;
        _rects->push_back(r);
    }   // end if
    else if ( checkAddRule( *this, r))
    {
        _areaSum += r.area();
        _intersection &= r;
        _union |= r;
        const int nrects = _rects->size();
        const int nrects1 = nrects+1;
        _mean.x = (_mean.x * nrects + r.x)/nrects1;
        _mean.y = (_mean.y * nrects + r.y)/nrects1;
        _mean.width = (_mean.width * nrects + r.width)/nrects1;
        _mean.height = (_mean.height * nrects + r.height)/nrects1;
        added = true;
        _rects->push_back(r);
    }   // end else

    return added;
}   // end add


// public
// returns value in (0,1]
double RectCluster::calcCompactness() const
{
    const cv::Point cp = getClusterCentre();
    const double sumArea = getAggregateArea();
    double sumDiffs = 0;
    double diff = 0;
    BOOST_FOREACH ( const cv::Rect& r, *_rects)
    {
        diff = sqrt(pow( r.x + r.width/2 - cp.x,2) + pow( r.y + r.height/2 - cp.y,2));
        // Weight the difference by the relative mass of the rectangle
        // (larger rectangles that are more distant are more important)
        diff *= double(r.area()) / sumArea;
        sumDiffs += diff;
    }   // end foreach
    sumDiffs /= _rects->size(); // Normalise by the number of rectangles
    return 1.0 / ( 1.0 + sumDiffs);
}   // end calcCompactness


// public
// returns value in [0,+inf)
double RectCluster::calcDensity() const
{
    const cv::Rect& urect = getUnion(); // Min enclosing rectangle
    if ( !urect.area())
        return 0;
    return double(getAggregateArea()) / urect.area();
}   // end calcDensity


// public
double RectCluster::calcQuality() const
{
    const double compactness = calcCompactness();
    const double density = calcDensity();
    const double clusterSize = sqrt(getUnion().area());
    return density * compactness * clusterSize;
}   // end calcQuality


// public
cv::Point RectCluster::getClusterCentre() const
{
    cv::Point cp(0,0);
    BOOST_FOREACH ( const cv::Rect& r, *_rects)
    {
        cp.x += r.x + r.width/2;
        cp.y += r.y + r.height/2;
    }   // end foreach
    cp.x = cvRound(float(cp.x) / (_rects->size()));
    cp.y = cvRound(float(cp.y) / (_rects->size()));
    return cp;
}   // end getClusterCentre


// public
bool RectCluster::operator<( const RectCluster& rc) const
{
    return calcQuality() < rc.calcQuality();
}   // end operator<


// public
void RFeatures::clusterRects( const std::list<cv::Rect>& boxes, float cap,
                              std::vector<RectCluster::Ptr>& clusters)
{
    BOOST_FOREACH ( const cv::Rect& box, boxes)
    {
        // Find which of the clusters, if any, box should be added to
        bool addedToCluster = false;
        BOOST_FOREACH ( RectCluster::Ptr& rc, clusters)
        {
            if ( rc->add( box))
            {
                addedToCluster = true;
                break;
            }   // end if
        }   // end foreach

        if ( !addedToCluster)
        {
            RectCluster::Ptr rc = RectCluster::create(cap);
            rc->add(box);
            clusters.push_back( rc);
        }   // end if
    }   // end foreach
}   // end clusterRects
