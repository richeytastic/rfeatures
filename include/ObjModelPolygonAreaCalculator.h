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

#ifndef RFEATURES_OBJ_MODEL_POLYGON_AREA_CALCULATOR_H
#define RFEATURES_OBJ_MODEL_POLYGON_AREA_CALCULATOR_H

#include "ObjModelTriangleMeshParser.h"

namespace RFeatures
{

class rFeatures_EXPORT ObjModelPolygonAreaCalculator : public ObjModelTriangleParser
{
public:
    ObjModelPolygonAreaCalculator();

    void reset();

    // Calculate and return the area of a polygon having the given vertices.
    static double calcFaceArea( const ObjModel::Ptr, int root, int a, int b);

    // Recalculate and return the area of the specified polygon on the underlying model.
    // (useful if the model has changed externally and don't want to redo whole model).
    double recalcPolygonArea( int fid);

    // These functions valid after parsing through ObjModelTriangleMeshParser (assuming all polys parsed)
    double getPolygonArea( int fid) const { return _polyAreas.at(fid);}

    // Check if the given poly was parsed.
    bool isPresent( int fid) const { return _polyAreas.count(fid) > 0;}

    void remove( int fid);  // Remove area information about this face

protected:
    virtual void parseTriangle( int fid, int uvroot, int uva, int uvb);

private:
    boost::unordered_map<int, double> _polyAreas;
    ObjModelPolygonAreaCalculator( const ObjModelPolygonAreaCalculator&);
    void operator=( const ObjModelPolygonAreaCalculator&);
};  // end class

}   // end namespace

#endif
