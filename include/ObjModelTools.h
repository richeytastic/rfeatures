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

#ifndef RFEATURES_OBJ_MODEL_TOOLS_H
#define RFEATURES_OBJ_MODEL_TOOLS_H

#include "CameraParams.h"
#include "DijkstraShortestPathFinder.h"     // A* search
#include "FeatureUtils.h"                   // Common miscellaneous/useful functions wrapping OpenCV functions.
#include "SurfaceCurveFinder.h"             // Find surface curvature following paths over model surfaces.
#include "Transformer.h"                    // Transform models and vertices in space.
#include "ObjModel.h"                       // Base 3D model type.
#include "ObjModelAligner.h"                // Use ICP or Procrustes to find a transform matrix to align models.
#include "ObjModelBounds.h"                 // Find the bounding box around a model or subsets of a model.
#include "ObjModelCopier.h"                 // Copies a model.
#include "ObjModelCurvatureMap.h"           // Calculates curvature information about manifolds.
#include "ObjModelCurvatureMetrics.h"       // Calculates useful curvature based metrics about a triangulated mesh.
#include "ObjModelEdgeFaceAdder.h"          
#include "ObjModelFastMarcher.h"            // Fast Marching to propagate distance maps along a triangulated manifold.
#include "ObjModelFunctionMapper.h"         // Generate a 3D object from a 2D height/function map.
#include "ObjModelHoleFiller.h"             // Fill holes on a model.
#include "ObjModelKDTree.h"                 // KD-tree for ObjModel.
#include "ObjModelKNNCorresponder.h"        // Correspond the vertices of a floating mesh to a target mesh using KNN.
#include "ObjModelManifolds.h"              // Finds components of a model as separate 2D manifolds.
#include "ObjModelMeshTraversalRecorder.h"  // Records polygons parsed using ObjModelTriangleMeshParser.
#include "ObjModelNormals.h"                // Calculate and store polygon normals.
#include "ObjModelOrienter.h"               // Use PCA to orient a model in space.
#include "ObjModelPolygonAngles.h"          // Calculate and store inner angles of the polygonal faces of models.
#include "ObjModelPolygonAreas.h"           // Calculate and store areas of polygons.
#include "ObjModelPolyUnfolder.h"           // Unfold a triangulated mesh into a plane.
#include "ObjModelRegionSelector.h"         // Select spherical sub-regions of a model.
#include "ObjModelReflector.h"              // Reflect model points through a given plane.
#include "ObjModelReferenceResampler.h"     // Resample a source mesh to a target mesh using minimal coregistration sets.
#include "ObjModelRemesher.h"               // Resample the mesh of an object.
#include "ObjModelSlicer.h"                 // Create a new model from the parts that lie on one half of a planar slice.
#include "ObjModelSmoother.h"               // Smooths joins between adjacent edges on a triangulated mesh.
#include "ObjModelPatchBendingEnergy.h"     // Calculate the bending energy using the 2D thin-splate spline model for point-sets.
#include "ObjModelSurfacePatches.h"         // Find vertices within a spherical region of arbitrary size.
#include "ObjModelSurfacePathFinder.h"      // Find the path over the surface of a model using arbitrary end-points.
#include "ObjModelSurfacePointFinder.h"     // Find the closest point on the surface of a model from an arbitrary location.
#include "ObjModelTetrahedronReplacer.h"    // Removes tetrahedrons from models.
#include "ObjModelTriangleMeshParser.h"     // Parse an ObjModel in such a way that adjacent polygon normals are on the same side.
#include "ObjModelWeights.h"
#include "ObjPolyInterpolator.h"            // Used with ObjModelRemesher.
#include "ObjPolyPlane.h"                   // Find where planes intersect with ObjPolys.

namespace RFeatures {

// Join the vertices and faces of the second ObjModel to the first.
// Material texture vectices are also mapped over if map txvrts = true.
rFeatures_EXPORT void join( ObjModel::Ptr, const ObjModel*, bool txvrts=true);

// Copy out and return the geometry of the parameter model with r units of vertex v.
rFeatures_EXPORT ObjModel::Ptr extractRadialArea( const ObjModel*, int v, float r);

// Copy out and return a subset of polygons connected at most N edges (min 1) from the given initial set of vertices.
rFeatures_EXPORT ObjModel::Ptr extractSubset( const ObjModel*, const IntSet&, int N=1);

// Removes all vertices not used in the definition of any triangles and returns the removed count.
rFeatures_EXPORT size_t removeDisconnectedVertices( ObjModel::Ptr);

// Given the shortest path between points v0 and v1 over the model, return
// the point x on that path that maximises d(v0 - x) + d(v1 - x) where d is the L2-norm.
rFeatures_EXPORT cv::Vec3f maximallyExtrudedPoint( const ObjModel*, int v0, int v1);

// Same as above but specifying points.
rFeatures_EXPORT cv::Vec3f maximallyExtrudedPoint( const ObjModelKDTree*, const cv::Vec3f& v0, const cv::Vec3f& v1);

// Given a vector of vertex IDs, return the index of the element in vids for
// the point x that maximises d(v0 - x) + d(v1 - x) where d is the L2-norm.
rFeatures_EXPORT int maximallyExtrudedPointIndex( const ObjModel*, const std::vector<int>& vids);

// Make and return a matrix with dimensionality 3 x n where n is the number of vertices in the given
// model. The given model must have its vertex IDs numbered sequentially.
rFeatures_EXPORT cv::Mat_<double> verticesToCvMat( const ObjModel*);

// Calculate and return the weighted mean column vector of A which must have dimensionality 3 x n.
// Column weights should be provided by the weights matrix which must be of dimension 1 x n.
// If weights is not provided (or there's a column count mismatch), the weights are all initialised to 1.
rFeatures_EXPORT cv::Vec3d calcMeanColumnVector( const cv::Mat_<double>& A, cv::Mat_<double> weights=cv::Mat_<double>());

// Translate the column vectors of A by subtracting the given vector m and return the post calculated root mean square distance.
rFeatures_EXPORT double toMean( cv::Mat_<double>& A, const cv::Vec3d& m, cv::Mat_<double> weights=cv::Mat_<double>());

using Triangle3f = cv::Vec3f[3];

// Obtain a set of basis vectors and magnitudes from the provided triangle space where uX is orthogonal to the plane,
// uY is a unit vector incident with T[2] - T[0], and uZ is defined as the cross product of uX with uY. The magnitudes
// mY and mZ are given according to the sqrt of the area of the triangle scaled by the projected lengths of the corresponding
// vectors along the basis vectors. Magnitude mX is simply the sqrt of the area of the triangle.
rFeatures_EXPORT void obtainBasis( const Triangle3f& T, cv::Vec3d& uX, double& mX, cv::Vec3d& uY, double& mY, cv::Vec3d& uZ, double& mZ);

// Transfer the relative relationship that point p has with triangle pSpace (point p defines the tip of a
// tetrahedron with pSpace as its base) to triangle qSpace, setting q as the relative point (point q defines the tip
// of the tetrahedron with qSpace as its base). The orthogonal distance of q from the plane defined by qSpace is returned.
// For this mapping to make sense, the corresponding indices of pSpace and qSpace must have some sort of semantic
// equivalence (i.e. be coregistered to semantically equivalent points).
rFeatures_EXPORT float mapPosition( const Triangle3f& pSpace, const cv::Vec3f& p, const Triangle3f& qSpace, cv::Vec3f& q);

}   // end namespace

#endif
