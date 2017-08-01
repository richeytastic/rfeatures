#ifndef RFEATURES_OBJ_MODEL_POLYGON_AREA_CALCULATOR_H
#define RFEATURES_OBJ_MODEL_POLYGON_AREA_CALCULATOR_H

#include "ObjModelTriangleMeshParser.h"

namespace RFeatures
{

class rFeatures_EXPORT ObjModelPolygonAreaCalculator : public ObjModelTriangleParser
{
public:
    explicit ObjModelPolygonAreaCalculator( const ObjModel::Ptr);

    void reset();

    // Calculate and return the area of a polygon having the given vertices.
    double operator()( int root, int a, int b) const;

    // Recalculate and return the area of the specified polyong on the underlying model.
    // (useful if the model has changed externally and don't want to redo whole model).
    double recalcPolygonArea( int fid);

    // These functions valid after parsing through ObjModelTriangleMeshParser
    double getPolygonArea( int fid) const { return _polyAreas.at(fid);}

    void remove( int fid);  // Remove area information about this face

protected:
    virtual void parseTriangle( int fid, int uvroot, int uva, int uvb);

private:
    const ObjModel::Ptr _model;
    boost::unordered_map<int, double> _polyAreas;
};  // end class

}   // end namespace

#endif
