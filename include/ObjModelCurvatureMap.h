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

#ifndef RFEATURES_OBJ_MODEL_CURVATURE_MAP_H
#define RFEATURES_OBJ_MODEL_CURVATURE_MAP_H

/**
 * Implements:
 * "Estimating the Tensor of Curvature of a Surface from a Polyhedral Approximation"
 * by Gabriel Taubin (1995).
 */

#include "ObjModelManifolds.h"

namespace RFeatures {

class rFeatures_EXPORT ObjModelCurvatureMap
{
public:
    using Ptr = std::shared_ptr<ObjModelCurvatureMap>;
    static Ptr create( const ObjModel&, const ObjModelManifolds&);

    // Update curvature for vertex vi. Call after position of vertex has changed.
    // Note that this also updates curvature of all connected vertices and faces.
    void update( const ObjModel&, const ObjModelManifolds&, int vi);

    // Cached face areas and normals.
    inline double faceArea( int fid) const { return _fareas.at(fid);}
    inline const cv::Vec3f& faceNorm( int fid) const { return _fnorms.at(fid);}

    // Get the principal curvature vectors tangent to the surface at vertex vi on manifold j.
    // On return, floats kp1 and kp2 are set to the corresponding curvature metrics.
    const cv::Vec3d& vertexPC1( int j, int vi, double &kp1) const;   // First principal component of curvature
    const cv::Vec3d& vertexPC2( int j, int vi, double &kp2) const;   // Second principal component of curvature

    // Get sum of the areas of the faces that have vi as a vertex on manifold j.
    double vertexAdjFacesSum( int j, int vi) const;    // Useful for weighting

    // Get the normal for the given vertex weighted by the areas of its adjacent polygons
    // on manifold j (adjacent polygons on other manifolds are not considered).
    // Larger polygons weight the normal more in the direction of that polygon.
    const cv::Vec3d& weightedVertexNormal( int j, int vi) const;

    // Calculate the normal for the given vertex weighted by the areas of its adjacent polygons
    // (from all manifolds). Larger polygons weight the normal more in the direction of that polygon.
    cv::Vec3f calcWeightedVertexNormal( const ObjModel&, int vi) const;

    // Given scalars a and b, compute c = cos(t) and s = sin(t) for some angle t so:
    // | c  s|t  |a|  =  |r|
    // |-s  c|   |b|     |0|
    // Returns r (which is always positive).
    static double calcGivensRotation( double a, double b, double& c, double& s);

private:
    std::unordered_map<int, double> _fareas;    // Areas of polygons
    std::unordered_map<int, cv::Vec3f> _fnorms; // Norms of polygons

    struct Curvature
    {
        cv::Vec3d T1, T2;
        double kp1, kp2;
    };  // end struct

    struct ManifoldData
    {
        explicit ManifoldData( const ObjModelCurvatureMap&);
        void map( const ObjModel&, const ObjModelManifolds&, int);

        std::unordered_map<int, cv::Vec3d> _vtxNormals;     // Normals at vertices
        std::unordered_map<int, double> _edgeFaceSums;      // Sum of face areas keyed by common edge ID
        std::unordered_map<int, double> _vtxAdjFacesSum;    // Sum of face areas keyed by common vertex ID
        std::unordered_map<int, Curvature> _vtxCurvature;   // Per vertex curvature

        void setWeightedVertexNormal( const ObjModel&, const ObjModelManifolds&, int, int);
        void setEdgeFaceSums( const ObjModel&, const ObjModelManifolds&, int, int);
        void setVertexAdjFaceSums( const ObjModel&, const ObjModelManifolds&, int, int);
        void setVertexCurvature( const ObjModel&, const ObjModelManifolds&, int, int);

    private:
        void addEdgeCurvature( const ObjModel&, const ObjModelManifolds&, int, int, int, cv::Matx33d&);
        const ObjModelCurvatureMap& _omcm;
    };  // end struct

    std::vector<ManifoldData*> _mdata;   // Manifold data

    void _updatePoly( const ObjModel&, const ObjModelManifolds&, int);
    void _map( const ObjModel&, const ObjModelManifolds&);
    ObjModelCurvatureMap();
    ~ObjModelCurvatureMap();
    ObjModelCurvatureMap( const ObjModelCurvatureMap&) = delete;
    void operator=( const ObjModelCurvatureMap&) = delete;
};  // end class

}   // end namespace

#endif
