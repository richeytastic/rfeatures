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
#include "FeatureUtils.h"                   // Common miscellaneous/useful functions wrapping OpenCV functions.
#include "Transformer.h"                    // Transform models and vertices in space.
#include "ObjModel.h"                       // Base 3D model type.
#include "ObjModelAligner.h"                // Use ICP to find a transform matrix to align models.
#include "ObjModelBoundaryFinder.h"         // Find 2D boundary edges.
#include "ObjModelCleaner.h"                // Cleans model to make a triangulated mesh.
#include "ObjModelComponentFinder.h"        // Finds disjoint connected components of a model.
#include "ObjModelCopier.h"                 // Copies a model.
#include "ObjModelCurvatureMap.h"           // Calculates curvature information about a single connected component.
#include "ObjModelCurvatureMetrics.h"       // Calculates useful curvature based metrics about a triangulated mesh.
#include "ObjModelEdgeFaceAdder.h"          
#include "ObjModelFastMarcher.h"            // Fast Marching to propagate distance maps along a triangulated manifold.
#include "ObjModelFunctionMapper.h"         // Generate a 3D object from a 2D height/function map.
#include "ObjModelGeodesicPathFinder.h"     // Find Geodesic paths over a surface (TODO).
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

#endif
