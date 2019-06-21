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

#ifndef RFEATURES_OBJ_MODEL_BOUNDS_H
#define RFEATURES_OBJ_MODEL_BOUNDS_H

/**
 * Calculate bounding space around a model or a subset of a model.
 */

#include "ObjModel.h"

namespace RFeatures {

class rFeatures_EXPORT ObjModelBounds
{
public:
    using Ptr = std::shared_ptr<ObjModelBounds>;
    static Ptr create( const ObjModel&, const IntSet *vset=nullptr);
    static Ptr create( const ObjModel&, const IntSet& fids);

    // Find the bounds of the given model over the given set of vertices (or all vertices if null).
    ObjModelBounds( const ObjModel&, const IntSet *vset=nullptr);

    // Find the bounds of the given model over the given set of faces.
    ObjModelBounds( const ObjModel&, const IntSet&);

    // Returns true iff the parameter bounds intersects with this one.
    bool intersects( const ObjModelBounds&) const;

    // Calculate and return the actual corner coordinates of the model bounds.
    void corners( cv::Vec3f& minc, cv::Vec3f& maxc) const;

    // Return the given corner bounds as: X_min, X_max, Y_min, Y_max, Z_min, Z_max.
    cv::Vec6d as6d() const;

    // Return the centre of the bounding box.
    cv::Vec3f centre() const;

    // Return the diagonal length between opposite corners.
    double diagonal() const;

    // Return the lengths of the three sides.
    double xlen() const;
    double ylen() const;
    double zlen() const;

    static cv::Vec6d as6d( const cv::Vec3f& minc, const cv::Vec3f& maxc);
    static cv::Vec3f centre( const cv::Vec3f& minc, const cv::Vec3f& maxc) { return 0.5f*(minc + maxc);}
    static double diagonal( const cv::Vec3f& minc, const cv::Vec3f& maxc) { return cv::norm( minc - maxc);}

private:
    const ObjModel& _model;
    cv::Vec6i _vbnd;    // Bounding box as vertex indices.
};  // end class

}   // end namespace

#endif
