#include "DistanceMeasurer.h"
using RFeatures::DistanceMeasurer;
#include <cassert>


DistanceMeasurer::DistanceMeasurer( const RFeatures::ObjModel::Ptr& om) : _om(om) {}


cv::Vec3f DistanceMeasurer::getMaximallyExtrudedPoint( int e0, int e1) const
{
    return getMaximallyExtrudedPoint( _om->getUniqueVertex(e0), _om->getUniqueVertex(e1));
}   // end getMaximallyExtrudedPoint


cv::Vec3f DistanceMeasurer::getMaximallyExtrudedPoint( const cv::Vec3f& v0, const cv::Vec3f& v1) const
{
    const int e0 = _om->lookupUniqueVertexIndex( v0);
    const int e1 = _om->lookupUniqueVertexIndex( v1);
    RFeatures::DijkstraShortestPathFinder dspf( _om);
    dspf.setEndPointUniqueVertexIndices( e0, e1);
    std::vector<int> uvids;
    dspf.findShortestPath( uvids);
    const int midx = getMaximallyExtrudedPointIndex( uvids);
    assert( midx >= 0 && midx < uvids.size());
    return _om->getUniqueVertex( uvids[midx]);
}   // end getMaximallyExtrudedPoint


int DistanceMeasurer::getMaximallyExtrudedPointIndex( const std::vector<int>& uvids) const
{
    const int npts = uvids.size();
    assert( npts > 0);
    if ( npts == 0)
        return -1;

    const cv::Vec3f v0 = _om->getUniqueVertex( uvids[0]);
    const cv::Vec3f v1 = _om->getUniqueVertex( uvids[npts-1]);
    int maxidx = -1;
    double maxvdist = 0;
    for ( int i = 0; i < npts; ++i)
    {
        const cv::Vec3f v = _om->getUniqueVertex(uvids[i]);
        const double vdist = cv::norm(v - v0) + cv::norm(v - v1);
        if ( vdist > maxvdist)
        {
            maxvdist = vdist;
            maxidx = i;
        }   // end if
    }   // end for

    return maxidx;
}   // end getMaximallyExtrudedPointIndex
