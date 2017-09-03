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

#include <ObjModelPolygonAreaCalculator.h>
#include <FeatureUtils.h>
using RFeatures::ObjModelPolygonAreaCalculator;
using RFeatures::ObjModel;

// public
ObjModelPolygonAreaCalculator::ObjModelPolygonAreaCalculator() {}

// public
void ObjModelPolygonAreaCalculator::reset()
{
    _polyAreas.clear();
}   // end reset


// public static
double ObjModelPolygonAreaCalculator::calcFaceArea( const ObjModel::Ptr model, int root, int a, int b)
{
    const cv::Vec3f& v0 = model->getVertex( root);
    const cv::Vec3f& v1 = model->getVertex( a);
    const cv::Vec3f& v2 = model->getVertex( b);
    return RFeatures::calcTriangleArea( v0, v1, v2); // Calculate by Heron's formula
}   // end calcFaceArea


// public
double ObjModelPolygonAreaCalculator::recalcPolygonArea( int fid)
{
    const int* vindices = model->getFaceVertices(fid);
    return _polyAreas[fid] = calcFaceArea( model, vindices[0], vindices[1], vindices[2]);
}   // end recalcPolygonArea


// public
void ObjModelPolygonAreaCalculator::remove( int fid)
{
    _polyAreas.erase(fid);
}   // end remove


// protected virtual
void ObjModelPolygonAreaCalculator::parseTriangle( int fid, int root, int a, int b)
{
    _polyAreas[fid] = calcFaceArea(model, root, a, b);
}   // end parseTriangle
