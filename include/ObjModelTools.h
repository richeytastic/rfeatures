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

#ifndef RFEATURES_OBJ_MODEL_TOOLS_H
#define RFEATURES_OBJ_MODEL_TOOLS_H

#include "CameraParams.h"
#include "DijkstraShortestPathFinder.h"     // A* search
#include "FeatureUtils.h"                   // Common miscellaneous/useful functions wrapping OpenCV functions.
#include "SurfaceCurveFinder.h"             // Find surface curvature following paths over model surfaces.
#include "Transformer.h"                    // Transform models and vertices in space.
#include "ObjModel.h"                       // Base 3D model type.
#include "ObjModelAligner.h"                // Use ICP or Procrustes to find a transform matrix to align models.
#include "ObjModelBoundaryFinder.h"         // Find 2D boundary edges.
#include "ObjModelCleaner.h"                // Cleans model to make a triangulated mesh.
#include "ObjModelComponentFinder.h"        // Finds disjoint connected components of a model.
#include "ObjModelCopier.h"                 // Copies a model.
#include "ObjModelCurvatureMap.h"           // Calculates curvature information about a single connected component.
#include "ObjModelCurvatureMetrics.h"       // Calculates useful curvature based metrics about a triangulated mesh.
#include "ObjModelEdgeFaceAdder.h"          
#include "ObjModelFastMarcher.h"            // Fast Marching to propagate distance maps along a triangulated manifold.
#include "ObjModelFunctionMapper.h"         // Generate a 3D object from a 2D height/function map.
#include "ObjModelHoleFiller.h"             // Fill holes on a model.
#include "ObjModelInfo.h"                   // Utility class to clean a model and wrap boundary and component info.
#include "ObjModelIntegrityChecker.h"       // Check the topology of a model to ensure it's a triangulated manifold.
#include "ObjModelKDTree.h"                 // KD-tree for ObjModel.
#include "ObjModelMeshTraversalRecorder.h"  // Records polygons parsed using ObjModelTriangleMeshParser.
#include "ObjModelNormals.h"                // Calculate and store polygon normals.
#include "ObjModelOrienter.h"               // Use PCA to orient a model in space.
#include "ObjModelPolygonAngles.h"          // Calculate and store inner angles of the polygonal faces of models.
#include "ObjModelPolygonAreas.h"           // Calculate and store areas of polygons.
#include "ObjModelPolygonCollapser.h"       // "Collapse" and remove arbitrary polygons from the mesh without leaving holes.
#include "ObjModelPolyUnfolder.h"           // Unfold a triangulated mesh into a plane.
#include "ObjModelRegionSelector.h"         // Select spherical sub-regions of a model.
#include "ObjModelReflector.h"              // Reflect model points through a given plane.
#include "ObjModelRemesher.h"               // Resample the mesh of an object.
#include "ObjModelSmoother.h"               // Smooths joins between adjacent edges on a triangulated mesh.
#include "ObjModelPatchBendingEnergy.h"     // Calculate the bending energy using the 2D thin-splate spline model for point-sets.
#include "ObjModelSurfacePatches.h"         // Find vertices within a spherical region of arbitrary size.
#include "ObjModelSurfacePathFinder.h"      // Find the path over the surface of a model using arbitrary end-points.
#include "ObjModelSurfacePointFinder.h"     // Find the closest point on the surface of a model from an arbitrary location.
#include "ObjModelTetrahedronReplacer.h"    // Removes tetrahedrons from models.
#include "ObjModelTopologyFinder.h"         // Parse the surface topology of an ObjModel (used by ObjModelIntegrityChecker).
#include "ObjModelTriangleMeshParser.h"     // Parse an ObjModel in such a way that adjacent polygon normals are on the same side.
#include "ObjModelVertexAdder.h"            // Subdivide the mesh further in various ways.
#include "ObjModelWeights.h"
#include "ObjPolyInterpolator.h"            // Used with ObjModelRemesher.

namespace RFeatures {

// Given the shortest path between points v0 and v1 over the model, return
// the point x on that path that maximises d(v0 - x) + d(v1 - x) where d is the L2-norm.
rFeatures_EXPORT cv::Vec3f maximallyExtrudedPoint( const ObjModel*, int v0, int v1);

// Same as above but specifying points.
rFeatures_EXPORT cv::Vec3f maximallyExtrudedPoint( const ObjModelKDTree*, const cv::Vec3f& v0, const cv::Vec3f& v1);

// Given a vector of vertex IDs, return the index of the element in vids for
// the point x that maximises d(v0 - x) + d(v1 - x) where d is the L2-norm.
rFeatures_EXPORT int maximallyExtrudedPointIndex( const ObjModel*, const std::vector<int>& vids);

// Given edge v0-->v1 on face fid, return the other shared face on edge v0-->v1 that ISN'T fid.
// If the edge shares no other faces, returns -1.
// ASSUMES NO MORE THAN TWO FACES PER EDGE!
rFeatures_EXPORT int oppositePoly( const ObjModel*, int fid, int v0, int v1);

// Create and return a matrix with m rows and n columns where m is 3 and n is the number of vertices in the provided model.
// The order of the column vectors matches the incrementing order of vertex IDs in the model. Note that the vertex IDs in
// the model will not necessarily correspond with the column index of the vector component in the returned matrix!
rFeatures_EXPORT cv::Mat_<double> verticesToCvMat( const ObjModel*);

// Create and return a matrix with 1 row and n columns where n is the number of entries in vw.
// The order of the column vectors matches the incrementing order of integer keys in vw. Note that the integer keys
// will not necessarily correspond with the column index of the component in the returned vector!
rFeatures_EXPORT cv::Mat_<double> weightsToCvMat( const VertexWeights& vw);

// Calculate and return the weighted mean column vector of A which must have dimensionality 3 x n.
// Column weights should be provided by the weights matrix which must be a 1 row, n column matrix with the same number
// of columns as A. If weights is not provided (or there's a column count mismatch), the weights are all initialised to 1.
rFeatures_EXPORT cv::Vec3d calcMeanColumnVector( const cv::Mat_<double>& A, cv::Mat_<double> weights=cv::Mat_<double>());

// Translate the column vectors of A by subtracting the given vector m and return the post calculated root mean square distance.
rFeatures_EXPORT double toMean( cv::Mat_<double>& A, const cv::Vec3d& m, cv::Mat_<double> weights=cv::Mat_<double>());

// Multiply column vectors of A by scaling factor s.
rFeatures_EXPORT void scale( cv::Mat_<double>& A, double s);
}   // end namespace

#endif
