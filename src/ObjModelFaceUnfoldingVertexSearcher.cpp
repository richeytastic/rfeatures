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

#include <ObjModelFaceUnfoldingVertexSearcher.h>
#include <ObjModelFaceAngleCalculator.h>
#include <ObjModelPolyUnfolder.h>
using RFeatures::ObjModelFaceUnfoldingVertexSearcher;
using RFeatures::ObjModelPolyUnfolder;
using RFeatures::ObjModel;
using RFeatures::ObjPoly;
#include <boost/foreach.hpp>
#include <iostream>
#include <cassert>


// Get the other face in fids that isn't fid. Returns fid if only fid present.
int getOther( const IntSet& fids, int fid)
{
    assert( fids.size() <= 2);
    BOOST_FOREACH ( int f, fids)
        if ( f != fid)
            return f;
    return fid;
}   // end if


// public
ObjModelFaceUnfoldingVertexSearcher::ObjModelFaceUnfoldingVertexSearcher( const ObjModel::Ptr m)
    : _model(m) {}


// public
int ObjModelFaceUnfoldingVertexSearcher::operator()( int ui, int T, cv::Vec3f& upos)
{
    const double theta = RFeatures::ObjModelFaceAngleCalculator::calcInnerAngle( _model, T, ui);
    return operator()( ui, T, theta, upos);
}   // end operator()


// public
int ObjModelFaceUnfoldingVertexSearcher::operator()( int ui, int T, double theta, cv::Vec3f& upos)
{
    static const double HALF_PI = CV_PI/2;
    assert( theta >= HALF_PI);

    ObjModelPolyUnfolder polyUnfolder( _model, T);

    const ObjPoly& face = _model->getFace( T);
    int uj, uk;
    face.getOpposite( ui, uj, uk);

    const cv::Vec3f& vi = _model->vtx( ui);   // The position of the search vector
    const cv::Vec3f& vj = _model->vtx( uj);
    const cv::Vec3f& vk = _model->vtx( uk);

    _cVec = vi;

    cv::Vec3d v0, v1;
    cv::normalize( vj - vi, v0);
    cv::normalize( vk - vi, v1);
    cv::normalize( v0 + v1, _secDirVec);
    // theta is defined to be the angle between v0 and v1
    // The valid section angle defined to be the allowed angle between _secDirVec and any candidate vector.
    _alpha = (CV_PI - theta)/2;

    _initEdgePos = vj;
    const cv::Vec3d initEdge = vk - vj;
    cv::normalize( initEdge, _initUnitEdge);
    _halfInitEdgeNorm = cv::norm( initEdge)/2;

    _recursionLim = 0;
    _parsedTriangles.clear();
    _parsedTriangles.insert(T);
    const int nextT = getOther( _model->getSharedFaces( uj, uk), T);
    const int ufound = searchForVertexInUnfoldingSection( &polyUnfolder, uj, uk, nextT);
    if ( ufound >= 0)
        upos = (cv::Vec3f)polyUnfolder.getUnfoldedVertex(ufound);

    return ufound;
}   // end operator()


// private
int ObjModelFaceUnfoldingVertexSearcher::searchForVertexInUnfoldingSection( ObjModelPolyUnfolder* polyUnfolder, int u0, int u1, int T)
{
    if ( _parsedTriangles.count(T) > 0)
        return -1;

    _parsedTriangles.insert(T);

    const int u2 = polyUnfolder->unfoldAlongEdge( T, u0, u1);
    const cv::Vec3d& v2 = polyUnfolder->getUnfoldedVertex( u2);

    // If the translated position of u2 (v2) is within the planar section search area, we're done!
    cv::Vec3d v2dir; // Direction of v2 from the initial search point
    cv::normalize( v2 - _cVec, v2dir);
    const double theta = acos( v2dir.dot(_secDirVec));  // Angle between the vectors
    if ( theta < _alpha)
        return u2;

    // Next triangle to be unfolded runs along edge u0->u2
    int ui = u0;
    int uj = u2;
    // Unless the better search area is the triangle along the other edge
    const double v2Proj = (v2 - _initEdgePos).dot(_initUnitEdge);
    if ( v2Proj < _halfInitEdgeNorm)  // Projection of edge e1 along the original edge
    {
        ui = u2;
        uj = u1;
    }   // end if

    // If still recursing after 1000 triangles, something is seriously wrong!
    _recursionLim++;
    if ( _recursionLim >= 1000)
    {
        std::cerr << "[WARNING] RFeatures::ObjModelFaceUnfoldingVertexSearcher::searchForVertexInUnfoldingSection: Exceeded recursion limit!" << std::endl;
        return -1;
    }   // end if

    const int nextT = getOther( polyUnfolder->getObject()->getSharedFaces( ui, uj), T);
    // Ordering of vertices ensures direction vectors calculated correctly.
    return searchForVertexInUnfoldingSection( polyUnfolder, ui, uj, nextT);
}   // end searchForVertexInUnfoldingSection
