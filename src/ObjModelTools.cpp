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

#include <ObjModelTools.h>
using RFeatures::ObjModelKDTree;
using RFeatures::ObjModel;
using RFeatures::Edge;
#include <cassert>


cv::Vec3f RFeatures::maximallyExtrudedPoint( const ObjModel* model, int e0, int e1)
{
    DijkstraShortestPathFinder dspf( model);
    dspf.setEndPointVertexIndices( e0, e1);
    std::vector<int> vidxs;
    dspf.findShortestPath( vidxs);
    const int midx = maximallyExtrudedPointIndex( model, vidxs);
    assert( midx >= 0 && midx < (int)vidxs.size());
    return model->vtx( vidxs[midx]);
}   // end maximallyExtrudedPoint


cv::Vec3f RFeatures::maximallyExtrudedPoint( const ObjModelKDTree* kdt, const cv::Vec3f& v0, const cv::Vec3f& v1)
{
    return maximallyExtrudedPoint( kdt->model(), kdt->find(v0), kdt->find(v1));
}   // end maximallyExtrudedPoint


int RFeatures::maximallyExtrudedPointIndex( const ObjModel* model, const std::vector<int>& vidxs)
{
    const int npts = int(vidxs.size());
    assert( npts > 0);
    if ( npts == 0)
        return -1;

    const cv::Vec3f v0 = model->vtx( vidxs[0]);
    const cv::Vec3f v1 = model->vtx( vidxs[npts-1]);
    int maxidx = -1;
    double maxvdist = 0;
    for ( int i = 0; i < npts; ++i)
    {
        const cv::Vec3f v = model->vtx(vidxs[i]);
        const double vdist = cv::norm(v - v0) + cv::norm(v - v1);
        if ( vdist > maxvdist)
        {
            maxvdist = vdist;
            maxidx = i;
        }   // end if
    }   // end for

    return maxidx;
}   // end maximallyExtrudedPointIndex


int RFeatures::oppositePoly( const ObjModel* model, int fid, int vi, int vj)
{
    const IntSet& sfids = model->getSharedFaces( vi, vj);
    if ( sfids.size() <= 1)
        return -1;

    assert( sfids.size() == 2);
    const int bfid = *sfids.begin();
    return bfid != fid ? bfid : *(++sfids.begin());
}   // end oppositePoly
