#ifndef RFEATURES_OBJ_MODEL_INTEGRITY_CHECKER_H
#define RFEATURES_OBJ_MODEL_INTEGRITY_CHECKER_H

#include "ObjModel.h"   // RFeatures

namespace RFeatures
{

enum ObjModelIntegrityError
{
    NO_INTEGRITY_ERROR,
    POLY_COUNT_MISMATCH,
    EDGE_COUNT_MISMATCH,
    VERTEX_POLY_CONNECTION_ERROR
};  // end enum


class rFeatures_EXPORT ObjModelIntegrityChecker
{
public:
    explicit ObjModelIntegrityChecker( const ObjModel::Ptr&);

    const ObjModel::Ptr getObject() const { return _model;}

    // Parses the provided model. Only if this function returns NO_INTEGRITY_ERROR
    // can the values from the accessor methods below be trusted.
    ObjModelIntegrityError checkIntegrity();

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

    void reset();
    bool checkIs2DManifold() const;

    ObjModelIntegrityChecker( const ObjModelIntegrityChecker&); // NO COPY
    void operator=( const ObjModelIntegrityChecker&);           // NO COPY
};  // end class

}   // end namespace

rFeatures_EXPORT std::ostream& operator<<( std::ostream&, const RFeatures::ObjModelIntegrityChecker&);

#endif
