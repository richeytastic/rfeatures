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

#include <ObjModelGeodesicPathFinder.h>
#include <DijkstraShortestPathFinder.h>
#include <ObjModelNormals.h>
#include <iostream>
using RFeatures::ObjModelGeodesicPathFinder;
using RFeatures::ObjModelNormals;
using RFeatures::ObjModelKDTree;
using RFeatures::ObjModel;

namespace {

// Calculate normal as the mean of the polygon normals connected to the given vertex.
cv::Vec3d calcNormal( const ObjModel* model, int vidx)
{
    const IntSet& sfids = model->getFaceIds( vidx);
    cv::Vec3d nvec(0,0,0);
    for ( int fid : sfids)
        nvec += ObjModelNormals::calcNormal( model, fid);
    cv::Vec3d ovec;
    cv::normalize( nvec, ovec); // Ensure normalised (can probably do in-place setting of nvec and don't need ovec)
    return ovec;
}   // end calcNormal


cv::Vec3f calcPathNormal( const ObjModel* model, int v0, int v1)
{
    const cv::Vec3d n0 = calcNormal( model, v0);
    const cv::Vec3d n1 = calcNormal( model, v1);
    cv::Vec3d nrm;  // As mean of path endpoint normals
    cv::normalize( n0 + n1, nrm);
    return (cv::Vec3f)nrm;
}   // end calcPathNormal


class GeodesicCostCalculator : public RFeatures::PathCostCalculator
{
public:
    GeodesicCostCalculator( const ObjModel* model, int v0, int v1)
        : _pnrm( calcPathNormal( model, v0, v1))
    {
    }   // end ctor

    virtual void initialiseEndPoints( const cv::Vec3f& v0, const cv::Vec3f& v1)
    {
        cv::normalize( v1-v0, _dvec);
        _pvec = _dvec.cross( _pnrm);
    }   // end initialiseEndPoints


    virtual double operator()( const cv::Vec3f& x, const cv::Vec3f& y) const
    {
        const cv::Vec3f xvec = x - y;
        // Only want magnitude in the plane _dvec, _pvec
        const double d0 = xvec.dot( _dvec);
        const double d1 = xvec.dot( _pvec);
        return d0*d0 + d1*d1;
    }   // end operator()

private:
    const cv::Vec3f _pnrm;
    cv::Vec3f _dvec;
    cv::Vec3f _pvec;
};  // end class

}   // end namespace


// public
ObjModelGeodesicPathFinder::ObjModelGeodesicPathFinder( const ObjModel* m) : _model(m), _kdtree(NULL)
{}   // end ctor


// public
ObjModelGeodesicPathFinder::ObjModelGeodesicPathFinder( const ObjModelKDTree* kd) : _model( kd->model()), _kdtree(kd)
{}   // end ctor


// public
int ObjModelGeodesicPathFinder::findGeodesic( const cv::Vec3f& v0, const cv::Vec3f& v1, std::vector<cv::Vec3f>& pts)
{
    if ( _kdtree == NULL)
    {
        std::cerr << "[ERROR] RFeatures::ObjModelGeodesicPathFinder::findGeodesic: KD-tree not available!" << std::endl;
        return -1;
    }   // end if

    const int u0 = _kdtree->find(v0);
    const int u1 = _kdtree->find(v1);
    return findGeodesic( u0, u1, pts);
}   // end findGeodesic


// public
int ObjModelGeodesicPathFinder::findGeodesic( int u0, int u1, std::vector<cv::Vec3f>& pts)
{
    GeodesicCostCalculator gcc( _model, u0, u1);
    RFeatures::DijkstraShortestPathFinder dspf( _model, &gcc);
    dspf.setEndPointVertexIndices( u0, u1);
    std::vector<int> vids;
    dspf.findShortestPath( vids);
    for ( int vid : vids)
        pts.push_back( _model->vtx(vid));
    return (int)vids.size();
}   // end findGeodesic
