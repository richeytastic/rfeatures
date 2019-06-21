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

#ifndef RFEATURES_OBJ_MODEL_SMOOTHER_H
#define RFEATURES_OBJ_MODEL_SMOOTHER_H

#include "ObjModelManifolds.h"
#include "ObjModelCurvatureMap.h"

namespace RFeatures {

class rFeatures_EXPORT ObjModelSmoother
{
public:
    ObjModelSmoother( ObjModel&,
                      ObjModelCurvatureMap&,
                      const ObjModelManifolds&);

    // Reinterpolate vertices having curvature greater than maxc. Prioritises highest curvature
    // vertices first. Smoothing may not result in all vertices having curvature <= maxc on return
    // since interpolation is local. Boundary vertices are not considered for smoothing.
    // Finishes when no vertices have curvature > maxc or when max number of iterations reached.
    void smooth( double maxc, size_t maxIterations=10);

private:
    ObjModel& _model;
    ObjModelCurvatureMap& _cmap;
    const ObjModelManifolds& _manf;

    void _adjustVertex( int, int);
    ObjModelSmoother( const ObjModelSmoother&) = delete;
    void operator=( const ObjModelSmoother&) = delete;
};  // end class

}   // end namespace

#endif
