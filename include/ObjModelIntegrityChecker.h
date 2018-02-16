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

#ifndef RFEATURES_OBJ_MODEL_INTEGRITY_CHECKER_H
#define RFEATURES_OBJ_MODEL_INTEGRITY_CHECKER_H

#include "ObjModel.h"   // RFeatures

namespace RFeatures
{

class rFeatures_EXPORT ObjModelIntegrityChecker
{
public:
    explicit ObjModelIntegrityChecker( const ObjModel::Ptr&);

    const ObjModel::Ptr getObject() const { return _model;}

    // Returns true if model integrity intact. Only if returns true
    // can the values from the accessor methods below be trusted.
    bool checkIntegrity();

    // After a call to checkIntegrity, use these functions to find out how
    // many unique vertices fulfill these connection conditions.
    int getNumFlat() const { return (int)_flat.size();}
    int getNumNonFlat() const { return (int)_nonFlat.size();}
    int getNumLine() const { return (int)_line.size();}
    int getNumUnconnected() const { return (int)_unconnected.size();}                // Lonely (without connections)
    int getNumFlatJunction() const { return (int)_flatJunction.size();}              // Joining separate polygons
    int getNumNonFlatJunctionAType() const { return (int)_nonFlatJunctionA.size();}  // Joining a 2D surface to edge polys
    int getNumNonFlatJunctionBType() const { return (int)_nonFlatJunctionB.size();}  // Joining two 2D surfaces
    int getNumEdge() const { return (int)_edges.size();}
    int getNumFlatEdge() const { return (int)_flatEdges.size();}

    bool is2DManifold() const { return _is2DManifold;}
    bool getIntegrity() const { return _integrity;}

    const IntSet& getFlat() const { return _flat;}
    const IntSet& getNonFlat() const { return _nonFlat;}
    const IntSet& getLine() const { return _line;}
    const IntSet& getUnconnected() const { return _unconnected;}
    const IntSet& getFlatJunction() const { return _flatJunction;}
    const IntSet& getNonFlatJunctionAType() const { return _nonFlatJunctionA;}
    const IntSet& getNonFlatJunctionBType() const { return _nonFlatJunctionB;}
    const IntSet& getEdge() const { return _edges;}
    const IntSet& getFlatEdge() const { return _flatEdges;}

private:
    const ObjModel::Ptr _model;
    IntSet _flat, _nonFlat, _unconnected, _line, _flatJunction, _nonFlatJunctionA, _nonFlatJunctionB, _edges, _flatEdges;
    bool _is2DManifold;
    bool _integrity;

    void reset();
    bool checkIs2DManifold() const;

    ObjModelIntegrityChecker( const ObjModelIntegrityChecker&); // NO COPY
    void operator=( const ObjModelIntegrityChecker&);           // NO COPY
};  // end class

}   // end namespace

rFeatures_EXPORT std::ostream& operator<<( std::ostream&, const RFeatures::ObjModelIntegrityChecker&);

#endif
