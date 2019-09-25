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

#include <ObjModelCurvatureMetrics.h>
#include <functional>
#include <cassert>
using RFeatures::ObjModelCurvatureMetrics;
using RFeatures::ObjModelManifolds;
using RFeatures::ObjModelCurvatureMap;
using RFeatures::ObjModel;
using RFeatures::ObjPoly;

namespace {

// Find the faces adjacent to fid on the manifold given by mpolys and set in sfids.
void findAdjacentFaces( const ObjModel& model, const IntSet& mpolys, int fid, IntSet& sfids)
{
    const ObjPoly &fc = model.face(fid);
    const IntSet& sf0 = model.spolys( fc[0], fc[1]);
    const IntSet& sf1 = model.spolys( fc[1], fc[2]);
    const IntSet& sf2 = model.spolys( fc[2], fc[0]);
    
    for ( int f : sf0)
    {
        if ( mpolys.count(f) > 0)
            sfids.insert(f);
    }   // end for

    for ( int f : sf1)
    {
        if ( mpolys.count(f) > 0)
            sfids.insert(f);
    }   // end for

    for ( int f : sf2)
    {
        if ( mpolys.count(f) > 0)
            sfids.insert(f);
    }   // end for

    sfids.erase(fid);
    assert( sfids.size() <= 3);
}   // end findAdjacentFaces


double vertexValue( const ObjModel& model, int vid, const std::function<double(int)>& fn)
{
    double v = 0;
    const IntSet& fids = model.faces(vid);
    for ( int fid : fids)
        v += fn(fid);
    return v / fids.size();
}   // end vertexValue

}   // end namespace


ObjModelCurvatureMetrics::ObjModelCurvatureMetrics( const ObjModel& model, const ObjModelManifolds& manf, const ObjModelCurvatureMap& cmap)
    : _model(model), _manf(manf), _cmap(cmap)
{
}   // end ctor


double ObjModelCurvatureMetrics::faceDeterminant( int fid) const
{
    const int j = _manf.manifoldId(fid);
    assert( j >= 0);
    const int* fvidxs = _model.fvidxs( fid);
    assert( fvidxs);
    const cv::Vec3d& n0 = _cmap.weightedVertexNormal( j, fvidxs[0]);
    const cv::Vec3d& n1 = _cmap.weightedVertexNormal( j, fvidxs[1]);
    const cv::Vec3d& n2 = _cmap.weightedVertexNormal( j, fvidxs[2]);
    return n2.dot(n0.cross(n1));   // Calculate determinant as the scalar triple product
}   // end faceDeterminant


double ObjModelCurvatureMetrics::vertexDeterminant( int vid) const
{
    return vertexValue( _model, vid, [this](int fid){ return faceDeterminant(fid);});
}   // end vertexDeterminant


double ObjModelCurvatureMetrics::faceKP1FirstOrder( int fid) const
{
    const int j = _manf.manifoldId(fid);
    assert( j >= 0);
    const int* fvidxs = _model.fvidxs( fid);
    assert( fvidxs);
    double ka, kb, kc;
    _cmap.vertexPC1( j, fvidxs[0], ka);
    _cmap.vertexPC1( j, fvidxs[1], kb);
    _cmap.vertexPC1( j, fvidxs[2], kc);
    // Face curvature is the average of the curvature at the 3 corner vertices. Note that these
    // curvatures have already been calculated using weights corresponding to the relative area
    // of this polygon with the sum of the area of the polygons connected to each of the vertices.
    return (ka + kb + kc)/3;
}   // end faceKP1FirstOrder


double ObjModelCurvatureMetrics::vertexKP1FirstOrder( int vid) const
{
    return vertexValue( _model, vid, [this](int fid){ return faceKP1FirstOrder(fid);});
}   // end vertexKP1FirstOrder


double ObjModelCurvatureMetrics::faceKP2FirstOrder( int fid) const
{
    const int j = _manf.manifoldId(fid);
    assert( j >= 0);
    const int* fvidxs = _model.fvidxs( fid);
    assert( fvidxs);
    double ka, kb, kc;
    _cmap.vertexPC2( j, fvidxs[0], ka);
    _cmap.vertexPC2( j, fvidxs[1], kb);
    _cmap.vertexPC2( j, fvidxs[2], kc);
    // Face curvature is the average of the curvature at the 3 corner vertices. Note that these
    // curvatures have already been calculated using weights corresponding to the relative area
    // of this polygon with the sum of the area of the polygons connected to each of the vertices.
    return (ka + kb + kc)/3;
}   // end faceKP2FirstOrder


double ObjModelCurvatureMetrics::vertexKP2FirstOrder( int vid) const
{
    return vertexValue( _model, vid, [this](int fid){ return faceKP2FirstOrder(fid);});
}   // end vertexKP2FirstOrder


double ObjModelCurvatureMetrics::faceKP1SecondOrder( int fid) const
{
    // The derivative of curvature is the difference in curvature between this face's
    // curvature and the curvature of its (up to) three adjacent neighbours.

    const IntSet& polys = _manf.manifold( _manf.manifoldId(fid))->polygons();
    IntSet adjf;
    findAdjacentFaces( _model, polys, fid, adjf);

    const double k = faceKP1FirstOrder(fid);
    double fdiff = 0.0;
    for ( int fs : adjf)
        fdiff += k - faceKP1FirstOrder( fs);

    return fdiff/adjf.size();
}   // end faceKP1SecondOrder


double ObjModelCurvatureMetrics::vertexKP1SecondOrder( int vid) const
{
    return vertexValue( _model, vid, [this](int fid){ return faceKP1SecondOrder(fid);});
}   // end vertexKP1SecondOrder


double ObjModelCurvatureMetrics::faceKP2SecondOrder( int fid) const
{
    // The derivative of curvature is the difference in curvature between this face's
    // curvature and the curvature of its (up to) three adjacent neighbours.

    const IntSet& polys = _manf.manifold( _manf.manifoldId(fid))->polygons();
    IntSet adjf;
    findAdjacentFaces( _model, polys, fid, adjf);
 
    const double k = faceKP2FirstOrder(fid);
    double fdiff = 0.0;
    for ( int fs : adjf)
        fdiff += k - faceKP2FirstOrder( fs);

    return fdiff/adjf.size();
}   // end faceKP2SecondOrder


double ObjModelCurvatureMetrics::vertexKP2SecondOrder( int vid) const
{
    return vertexValue( _model, vid, [this](int fid){ return faceKP2SecondOrder(fid);});
}   // end vertexKP2SecondOrder
