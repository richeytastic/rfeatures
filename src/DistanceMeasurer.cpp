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

#include <DistanceMeasurer.h>
using RFeatures::DijkstraShortestPathFinder;
using RFeatures::DistanceMeasurer;
using RFeatures::ObjModel;
#include <cassert>


DistanceMeasurer::DistanceMeasurer( const ObjModel* om) : _om(om) {}


cv::Vec3f DistanceMeasurer::getMaximallyExtrudedPoint( int e0, int e1) const
{
    return getMaximallyExtrudedPoint( _om->getVertex(e0), _om->getVertex(e1));
}   // end getMaximallyExtrudedPoint


cv::Vec3f DistanceMeasurer::getMaximallyExtrudedPoint( const cv::Vec3f& v0, const cv::Vec3f& v1) const
{
    const int e0 = _om->lookupVertexIndex( v0);
    const int e1 = _om->lookupVertexIndex( v1);
    DijkstraShortestPathFinder dspf( _om);
    dspf.setEndPointVertexIndices( e0, e1);
    std::vector<int> vidxs;
    dspf.findShortestPath( vidxs);
    const int midx = getMaximallyExtrudedPointIndex( vidxs);
    assert( midx >= 0 && midx < (int)vidxs.size());
    return _om->getVertex( vidxs[midx]);
}   // end getMaximallyExtrudedPoint


int DistanceMeasurer::getMaximallyExtrudedPointIndex( const std::vector<int>& vidxs) const
{
    const int npts = (int)vidxs.size();
    assert( npts > 0);
    if ( npts == 0)
        return -1;

    const cv::Vec3f v0 = _om->getVertex( vidxs[0]);
    const cv::Vec3f v1 = _om->getVertex( vidxs[npts-1]);
    int maxidx = -1;
    double maxvdist = 0;
    for ( int i = 0; i < npts; ++i)
    {
        const cv::Vec3f v = _om->getVertex(vidxs[i]);
        const double vdist = cv::norm(v - v0) + cv::norm(v - v1);
        if ( vdist > maxvdist)
        {
            maxvdist = vdist;
            maxidx = i;
        }   // end if
    }   // end for

    return maxidx;
}   // end getMaximallyExtrudedPointIndex
