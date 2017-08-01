#include "DistanceMeasurer.h"
using RFeatures::DistanceMeasurer;
#include <cassert>


DistanceMeasurer::DistanceMeasurer( const RFeatures::ObjModel::Ptr& om) : _om(om) {}


cv::Vec3f DistanceMeasurer::getMaximallyExtrudedPoint( int e0, int e1) const
{
    return getMaximallyExtrudedPoint( _om->getVertex(e0), _om->getVertex(e1));
}   // end getMaximallyExtrudedPoint


cv::Vec3f DistanceMeasurer::getMaximallyExtrudedPoint( const cv::Vec3f& v0, const cv::Vec3f& v1) const
{
    const int e0 = _om->lookupVertexIndex( v0);
    const int e1 = _om->lookupVertexIndex( v1);
    RFeatures::DijkstraShortestPathFinder dspf( _om);
    dspf.setEndPointVertexIndices( e0, e1);
    std::vector<int> uvids;
    dspf.findShortestPath( uvids);
    const int midx = getMaximallyExtrudedPointIndex( uvids);
    assert( midx >= 0 && midx < uvids.size());
    return _om->getVertex( uvids[midx]);
}   // end getMaximallyExtrudedPoint


int DistanceMeasurer::getMaximallyExtrudedPointIndex( const std::vector<int>& uvids) const
{
    const int npts = (int)uvids.size();
    assert( npts > 0);
    if ( npts == 0)
        return -1;

    const cv::Vec3f v0 = _om->getVertex( uvids[0]);
    const cv::Vec3f v1 = _om->getVertex( uvids[npts-1]);
    int maxidx = -1;
    double maxvdist = 0;
    for ( int i = 0; i < npts; ++i)
    {
        const cv::Vec3f v = _om->getVertex(uvids[i]);
        const double vdist = cv::norm(v - v0) + cv::norm(v - v1);
        if ( vdist > maxvdist)
        {
            maxvdist = vdist;
            maxidx = i;
        }   // end if
    }   // end for

    return maxidx;
}   // end getMaximallyExtrudedPointIndex
