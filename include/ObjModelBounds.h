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
 * Oriented bounding cuboid around a model or a subset of a model.
 */

#include "ObjModel.h"

namespace RFeatures {

class rFeatures_EXPORT ObjModelBounds
{
public:
    using Ptr = std::shared_ptr<ObjModelBounds>;
    static Ptr create( const cv::Vec3d& minCorner, const cv::Vec3d& maxCorner, const cv::Matx44d& m=cv::Matx44d::eye());
    static Ptr create( const ObjModel&, const cv::Matx44d& m=cv::Matx44d::eye(), const IntSet *vset=nullptr);
    static Ptr create( const ObjModel&, const cv::Matx44d&, const IntSet& fids);

    /**
     * Set the bounding corners of a cuboid directly. The corners represent the cuboid
     * in its transformed position due to the provided matrix. That is, applying the inverse
     * of m to the corner vertices should give the cuboid region in standard position.
     */
    ObjModelBounds( const cv::Vec3d& minCorner, const cv::Vec3d& maxCorner, const cv::Matx44d& m=cv::Matx44d::eye());

    /**
     * Find the bounds of the given model over the given set of vertices (or all vertices if null).
     * The matrix gives the orientation of the model and the bounds are computed with respect to this.
     * Note that this is a semantically different transformation matrix than the model's own transform
     * which is used only for its efficient transformation without updating vertices. Calculation of the
     * bounds proceeds by finding the upright bounds by checking each vertex of the model transformed by
     * the inverse of the provided matrix, then setting the transform for these bounds to the provided matrix.
     */
    ObjModelBounds( const ObjModel&, const cv::Matx44d&, const IntSet *vset=nullptr);

    /**
     * Find the bounds of the given model over the given set of faces.
     */
    ObjModelBounds( const ObjModel&, const cv::Matx44d&, const IntSet&);

    /**
     * Copy constructors.
     */
    ObjModelBounds( const ObjModelBounds&) = default;
    ObjModelBounds& operator=( const ObjModelBounds&) = default;
    Ptr deepCopy() const;

    /**
     * Set (overwriting) the existing transform to specify the position and orientation of these bounds.
     */
    void setTransformMatrix( const cv::Matx44d&);

    /**
     * Set the transform by adding a transform after the existing one. If the existing
     * transform is the identity matrix then this is equivalent to setTransform.
     */
    void addTransformMatrix( const cv::Matx44d&);

    /**
     * Return the transform matrix being applied to the bounds.
     */
    const cv::Matx44d& transformMatrix() const { return _tmat;}

    /**
     * Rebaseline the corners of the cuboid according to the transformed corners (i.e. fix in place
     * the transformed corners), and reset the internal transform matrix to the identity matrix.
     */
    void fixTransformMatrix();

    /**
     * Make this ObjModelBounds a union with the given bounds so that on return this object
     * has bounds that encompass the parameter bounds (note that the transform matrix on the
     * parameter bounds may be different to this one's).
     */
    void encompass( const ObjModelBounds&);

    /**
     * Returns true iff the parameter bounds intersects with this one.
     */
    bool intersects( const ObjModelBounds&) const;

    /**
     * Return the transformed min and max corners of this bounding region.
     */
    cv::Vec3d minCorner() const;
    cv::Vec3d maxCorner() const;

    /**
     * Return the min and max corners representing the cuboid region containing the transformed
     * bounds. These corners change as the transform matrix is updated.
     */
    inline const cv::Vec3d& minExtent() const { return _mine;}
    inline const cv::Vec3d& maxExtent() const { return _maxe;}

    /**
     * Return the centre of the bounding box.
     */
    cv::Vec3d centre() const;

    /**
     * Return the lengths of the three sides of the cuboid.
     */
    double xlen() const;
    double ylen() const;
    double zlen() const;

    /**
     * Return the diagonal distance.
     */
    double diagonal() const;
        
    /**
     * Return the UNTRANSFORMED corners as: X_min, X_max, Y_min, Y_max, Z_min, Z_max.
     */
    cv::Vec6d cornersAs6d() const;

    /**
     * Return the extents as: X_min, X_max, Y_min, Y_max, Z_min, Z_max.
     */
    cv::Vec6d extentsAs6d() const;

    /**
     * Return the two corners as a single vector with elements: X_min, X_max, Y_min, Y_max, Z_min, Z_max.
     */
    static cv::Vec6d as6d( const cv::Vec3d& minc, const cv::Vec3d& maxc);

private:
    cv::Vec3d _minc, _maxc; // Min/max corners
    cv::Vec3d _mine, _maxe; // Min/max extents
    cv::Matx44d _tmat, _imat;
    void _init( const ObjModel&, const cv::Matx44d&, const IntSet&);
    void _calcExtents();
};  // end class


}   // end namespace

#endif
