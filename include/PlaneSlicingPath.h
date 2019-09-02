/************************************************************************
 * Copyright (C) 2019 Richard Palmer
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

#ifndef RFEATURES_PLANE_SLICING_PATH_H
#define RFEATURES_PLANE_SLICING_PATH_H

/**
 * Helper class for ObjModelSurfacePlanePathFinder.
 */

#include <ObjModel.h>
#include <deque>

namespace RFeatures {

class rFeatures_EXPORT PlaneSlicingPath
{
public:
    // The starting point must be inside polygon infid.
    PlaneSlicingPath( const ObjModel&, int infid, const cv::Vec3f&);
    virtual ~PlaneSlicingPath(){}

    // There are two directions to go in - specify the next face in the path
    // by specifying the face that SHOULDN'T be next.
    void init( int notMeFaceId=-1);

    int initPoly() const { return _ifid;}
    int firstPoly() const { return _ffid;}
    int nextPoly() const;

    bool canSplice( const PlaneSlicingPath&) const;

    void splice( const PlaneSlicingPath&, std::vector<cv::Vec3f>&) const;

    // Returns true iff this path can be extended at one or both of its ends.
    bool canExtend() const;

    // Returns true if can keep extending the front or back of the path.
    bool extend();

protected:
    virtual cv::Vec3f polySlicingPlane( int fid, const cv::Vec3f&) const = 0;

    inline const ObjModel& model() const { return _model;}

    inline const cv::Vec3f& lastVertexAdded() const { return _evtxs.back();}

private:
    const ObjModel& _model;
    const cv::Vec3f _ip;    // The initial vertex (inside the initial polygon)
    const int _ifid;        // Initial polygon
    int _ffid;              // The first polygon after the initial (might be -1)
    int _nfid;              // Next polygon id
    IntSet _pfids;
    std::deque<cv::Vec3f> _evtxs;    // Edge crossing vertices

    void _pushOnInitialToBack( std::vector<cv::Vec3f>& path) const;
    void _pushOnBackToInitial( std::vector<cv::Vec3f>& path) const;
    int _findNextPolyEdgeVertex( int, const cv::Vec3f&, cv::Vec3f&);
};  // end class

}   // end namespace

#endif
