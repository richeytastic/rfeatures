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

#ifndef RFEATURES_OBJ_MODEL_SMOOTHER_H
#define RFEATURES_OBJ_MODEL_SMOOTHER_H

#include "ObjModelCurvatureMap.h"
#include <ProgressDelegate.h> // rlib

namespace RFeatures {

class rFeatures_EXPORT ObjModelSmoother
{
public:
    // All given structures will be updated upon smoothing.
    ObjModelSmoother( ObjModel::Ptr,
                      ObjModelCurvatureMap::Ptr,
                      ObjModelNormals&,
                      ObjModelPolygonAreas&,
                      rlib::ProgressDelegate* pd=NULL);

    // Reinterpolate vertices having curvature greater than maxc. Prioritises highest curvature
    // vertices first. Smoothing may not result in all vertices having curvature <= maxc on return
    // since interpolation is local. Multiple iterations may be required to obtain the desired
    // degree of smoothing. Boundary vertices are not considered for smoothing. Algorithm finishes
    // when no vertices have curvature > maxc or when the max number of iterations has been reached.
    void smooth( double maxc, size_t maxIterations=10);

private:
    ObjModel::Ptr _model;
    ObjModelCurvatureMap::Ptr _cmap;
    ObjModelNormals& _normals;
    ObjModelPolygonAreas& _pareas;
    rlib::ProgressDelegate *_progressDelegate;

    void adjustVertex(int);
    ObjModelSmoother( const ObjModelSmoother&); // No copy
    void operator=( const ObjModelSmoother&);   // No copy
};  // end class

}   // end namespace

#endif
