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

#include <ObjModelPolygonAreas.h>
#include <FeatureUtils.h>
using RFeatures::ObjModelPolygonAreas;
using RFeatures::ObjModel;

// public
ObjModelPolygonAreas::ObjModelPolygonAreas() {}

// public
void ObjModelPolygonAreas::reset()
{
    _polyAreas.clear();
}   // end reset


// public static
double ObjModelPolygonAreas::calcFaceArea( const ObjModel* m, int fidx)
{
    const int* vidxs = m->getFaceVertices(fidx);
    if ( !vidxs)
        return 0.0;
    return calcFaceArea( m, vidxs[0], vidxs[1], vidxs[2]);
}   // end calcFaceArea


// public static
double ObjModelPolygonAreas::calcFaceArea( const ObjModel* m, int root, int a, int b)
{
    return RFeatures::calcTriangleArea( m->vtx( root), m->vtx( a), m->vtx( b));
}   // end calcFaceArea


// public
double ObjModelPolygonAreas::recalcPolygonArea( int fid)
{
    const int* vindices = model->getFaceVertices(fid);
    if ( !vindices)
        return 0.0;
    return _polyAreas[fid] = calcFaceArea( model, vindices[0], vindices[1], vindices[2]);
}   // end recalcPolygonArea


// public
void ObjModelPolygonAreas::remove( int fid)
{
    _polyAreas.erase(fid);
}   // end remove


// protected virtual
void ObjModelPolygonAreas::parseTriangle( int fid, int root, int a, int b)
{
    _polyAreas[fid] = calcFaceArea(model, root, a, b);
}   // end parseTriangle
