#ifndef RFEATURES_OBJ_MODEL_CURVATURE_MAP_H
#define RFEATURES_OBJ_MODEL_CURVATURE_MAP_H

/**
 * Implements:
 * "Estimating the Tensor of Curvature of a Surface from a Polyhedral Approximation"
 * by Gabriel Taubin (1995).
 *
 * Richard Palmer 2017
 */

#include "ObjModel.h"

namespace RFeatures
{

class ObjModelPolygonAreaCalculator;
class ObjModelNormalCalculator;


class rFeatures_EXPORT ObjModelCurvatureMap
{
public:
    typedef boost::shared_ptr<ObjModelCurvatureMap> Ptr;

    // sfid: The starting polygon ID for parsing the given model. This choice of polygon defines
    // the normal direction for all polygons. Polygon sfid has its normal calculated to have a
    // positive dot product with the +ve Z axis.
    static Ptr create( ObjModel::Ptr, int sfid);

    ObjModel::Ptr getObject() { return _model;} // Changes to the returned object may invalidate this curvature map!
    const ObjModel::Ptr getObject() const { return _model;}

    void recalcVertex( int vidx); // Recalculate curvature around the given vertex (including normals for the polygons using this vertex).
    void recalcFace( int fid);    // Recalculate face normal and curvature of attached vertices.

    // Get the principal curvature vectors tangent to the surface at vertex vi.
    // On return, floats kp1 and kp2 are set to the corresponding curvature metrics.
    const cv::Vec3d& getVertexPrincipalCurvature1( int vi, double &kp1) const;   // Maximum curvature
    const cv::Vec3d& getVertexPrincipalCurvature2( int vi, double &kp2) const;   // Minumum curvature

    // Can use these functions to weight values from a given vertex in a particular face "direction".
    double getFaceArea( int fid) const; // Get the area of the given face.
    // Get the sum of the areas of the faces that have vi as one of their vertices.
    double getVertexAdjFacesSum( int vi) const;

    const cv::Vec3d& getVertexNormal( int vi) const;
    const cv::Vec3d& getFaceNormal( int fid) const;

    // Given scalars a and b, compute c = cos(t) and s = sin(t) for some angle t so:
    // | c  s|t  |a|  =  |r|
    // |-s  c|   |b|     |0|
    // Returns r (which is always positive).
    static double calcGivensRotation( double a, double b, double& c, double& s);

private:
    ObjModel::Ptr _model;
    ObjModelPolygonAreaCalculator *_faceAreas;          // Per face areas
    ObjModelNormalCalculator *_faceNorms;               // Per face normals
    boost::unordered_map<int, cv::Vec3d> _vtxNormals;   // Normals at vertices
    boost::unordered_map<int, IntSet> _vtxEdgeIds;      // Edge IDs keyed by vertex ID
    boost::unordered_map<int, double> _edgeFaceSums;    // Sum of face areas keyed by common edge ID
    boost::unordered_map<int, double> _vtxAdjFacesSum;  // Sum of face areas keyed by common vertex ID

    struct Curvature
    {
        cv::Vec3d T1, T2;
        double kp1, kp2;
    };  // end struct
    boost::unordered_map<int, Curvature> _vtxCurvature;

    void calcVertexNormal( int);
    void calcEdgeFaceSums( int);
    void calcVertexAdjFaceSums( int);
    void calcVertexCurvature( int vi);
    void addEdgeCurvature( int, int, cv::Matx33d&);

    ObjModelCurvatureMap( ObjModel::Ptr, int sfid);
    virtual ~ObjModelCurvatureMap();
    ObjModelCurvatureMap( const ObjModelCurvatureMap&);     // No copy
    void operator=( const ObjModelCurvatureMap&);           // No copy
    class Deleter;
};  // end class

}   // end namespace

#endif
