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
    ObjModelSmoother( ObjModelCurvatureMap::Ptr curvMap); // Given curvature map is updated afterwards.

    // Reinterpolate vertices where they have curvature greater than maxc.
    // Smoothing prioritises highest curvature vertices. Smoothing will not
    // necessarily result in all vertices having curvature <= maxc on return
    // since interpolation is only local. Multiple iterations may be required
    // to obtain the desired degree of smoothing. Returns the highest curvature
    // on a vertex post smoothing (which may be higher than maxc). Boundary
    // vertices are NOT included in the smoothing operation by default.
    // If fewer than numIterations are needed to adjust all of the vertices to
    // have curvature <= maxc, on return numIterations is set to the number needed.
    double smooth( double maxc, size_t& numIterations, bool includeBoundary=false);

    void setProgressDelegate( rlib::ProgressDelegate*);

private:
    ObjModelCurvatureMap::Ptr _curvMap;
    rlib::ProgressDelegate *_progressDelegate;
};  // end class

}   // end namespace

#endif
