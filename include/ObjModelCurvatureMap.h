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

#ifndef RFEATURES_OBJ_MODEL_CURVATURE_MAP_H
#define RFEATURES_OBJ_MODEL_CURVATURE_MAP_H

/**
 * Implements:
 * "Estimating the Tensor of Curvature of a Surface from a Polyhedral Approximation"
 * by Gabriel Taubin (1995).
 *
 * Richard Palmer 2017
 */

#include "ObjModelNormals.h"
#include "ObjModelPolygonAreas.h"

namespace RFeatures {

class rFeatures_EXPORT ObjModelCurvatureMap
{
public:
    typedef std::shared_ptr<ObjModelCurvatureMap> Ptr;
    static Ptr create( const ObjModel*, const ObjModelNormals*, const ObjModelPolygonAreas*);

    const ObjModel* model() const { return _model;}
    const ObjModelNormals* normals() const { return _normals;}
    const ObjModelPolygonAreas* areas() const { return _pareas;}

    // Map curvature to the set of vertices given. Normal and area data must already be
    // present in the ObjModelNormals and ObjModelPolygonAreas objects respectively for
    // the polygons associated with the given vertices. Note that the set of vertices
    // comprising the whole surface that curvature needs to be calculated for should be
    // provided since curvature for a vertex is calculated from information about the
    // polygons connected to it.
    void map( const IntSet&);

    // Get the principal curvature vectors tangent to the surface at vertex vi.
    // On return, floats kp1 and kp2 are set to the corresponding curvature metrics.
    const cv::Vec3d& vertexPC1( int vi, double &kp1) const;   // First principal component of curvature
    const cv::Vec3d& vertexPC2( int vi, double &kp2) const;   // Second principal component of curvature

    // Get sum of the areas of the faces that have vi as a vertex.
    double vertexAdjFacesSum( int vi) const;    // Useful for weighting

    // Get the normal for the given vertex weighted by the areas of its adjacent polygons.
    // Larger polygons weight the normal more in the direction of that polygon.
    const cv::Vec3d& weightedVertexNormal( int vi) const;

    // Given scalars a and b, compute c = cos(t) and s = sin(t) for some angle t so:
    // | c  s|t  |a|  =  |r|
    // |-s  c|   |b|     |0|
    // Returns r (which is always positive).
    static double calcGivensRotation( double a, double b, double& c, double& s);

private:
    const ObjModel *_model;
    const ObjModelNormals *_normals;           // Per face normals
    const ObjModelPolygonAreas *_pareas;       // Per face areas
    std::unordered_map<int, cv::Vec3d> _vtxNormals;     // Normals at vertices
    std::unordered_map<int, IntSet> _vtxEdgeIds;        // Edge IDs keyed by vertex ID
    std::unordered_map<int, double> _edgeFaceSums;      // Sum of face areas keyed by common edge ID
    std::unordered_map<int, double> _vtxAdjFacesSum;    // Sum of face areas keyed by common vertex ID

    struct Curvature
    {
        cv::Vec3d T1, T2;
        double kp1, kp2;
    };  // end struct
    std::unordered_map<int, Curvature> _vtxCurvature;

    void setWeightedVertexNormal( int);
    void setEdgeFaceSums( int);
    void setVertexAdjFaceSums( int);
    void setVertexCurvature( int);
    void addEdgeCurvature( int, int, cv::Matx33d&);

    ObjModelCurvatureMap( const ObjModel*, const ObjModelNormals*, const ObjModelPolygonAreas*);
    ObjModelCurvatureMap( const ObjModelCurvatureMap&);     // No copy
    void operator=( const ObjModelCurvatureMap&);           // No copy
};  // end class

}   // end namespace

#endif
