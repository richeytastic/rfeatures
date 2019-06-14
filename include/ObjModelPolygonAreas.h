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

#ifndef RFEATURES_OBJ_MODEL_POLYGON_AREAS_H
#define RFEATURES_OBJ_MODEL_POLYGON_AREAS_H

#include "ObjModelTriangleMeshParser.h"

namespace RFeatures {

class rFeatures_EXPORT ObjModelPolygonAreas : public ObjModelTriangleParser
{
public:
    ObjModelPolygonAreas();

    void reset() override;

    // Calculate and return polygon area having given vertices (by Heron's).
    static double calcFaceArea( const ObjModel*, int fidx);
    static double calcFaceArea( const ObjModel*, int root, int a, int b);

    // Recalculate, set in this object and return the area of the specified polygon on
    // the model. Call if model has changed and don't want to reparse the whole model.
    double recalcPolygonArea( int fid);

    // These functions valid after parsing through ObjModelTriangleMeshParser (assuming all polys parsed)
    double area( int fid) const { return _polyAreas.at(fid);}

    // Check if the given poly was parsed.
    bool isPresent( int fid) const { return _polyAreas.count(fid) > 0;}

    void remove( int fid);  // Remove area information about this face

protected:
    void parseTriangle( int fid, int uvroot, int uva, int uvb) override;

private:
    std::unordered_map<int, double> _polyAreas;

    ObjModelPolygonAreas( const ObjModelPolygonAreas&) = delete;
    void operator=( const ObjModelPolygonAreas&) = delete;
};  // end class

}   // end namespace

#endif
