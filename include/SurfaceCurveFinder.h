/************************************************************************
 * Copyright (C) 2018 Richard Palmer
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

#ifndef RFEATURES_SURFACE_CURVE_FINDER_H
#define RFEATURES_SURFACE_CURVE_FINDER_H

#include "ObjModel.h"

namespace RFeatures {

class rFeatures_EXPORT SurfaceCurveFinder
{
public:
    explicit SurfaceCurveFinder( const ObjModel*);

    const ObjModel* model() const { return _model;}

    // Creates a geodesic between v0 and v1 where the orientation of the curve is determined
    // by the local curvature of every polygon crossed between the two points.
    // Returns whether a direct path was found or false if an unbroken sequence of polygons
    // between the two points could not be found.
    bool findPath( const cv::Vec3f& v0, int sT, const cv::Vec3f& v1, int fT, std::list<cv::Vec3f>& points);

private:
    const ObjModel *_model;
    int getOppositeEdge( const cv::Vec3d&, const cv::Vec3d&, int) const;

    SurfaceCurveFinder( const SurfaceCurveFinder&) = delete;
    void operator=( const SurfaceCurveFinder&) = delete;
};  // end class

}   // end namespace

#endif
