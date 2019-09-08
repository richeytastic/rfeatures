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

#ifndef RFEATURES_OBJ_MODEL_KD_TREE_H
#define RFEATURES_OBJ_MODEL_KD_TREE_H

#include "ObjModel.h"

namespace RFeatures {

class rFeatures_EXPORT ObjModelKDTree
{
public:
    using Ptr = std::shared_ptr<ObjModelKDTree>;

    /**
     * Create a new ObjModelKDTree from the untransformed vertices of the given model.
     * The initial transform matrix is set as the model's transformation matrix.
     * Client should keep the two in sync (see setTransformMatrix).
     */
    static Ptr create( const ObjModel&);

    /**
     * Return the number of vertices stored in the tree.
     */
    size_t numVtxs() const;

    /**
     * Find the ID of the model vertex closest to p. The returned vertex ID is
     * valid for the model used to construct this ObjModelKDTree. If not null,
     * set sqdis on return to be the squared distance to the vertex.
     */
    int find( const cv::Vec3f& p, float *sqdis=nullptr) const;
    int find( const cv::Vec3d& p, float *sqdis=nullptr) const;

    /**
     * Find the closest n vertices on the model that are closest to p. n is specified by
     * preallocating nvidxs to the desired size. If sqdis is given, it must also have
     * the same size as nvidxs. Returns actual number of points found which may be less than n.
     */
    int findn( const cv::Vec3f& p, std::vector<int>& nvidxs, std::vector<float>* sqdis=nullptr) const;
    int findn( const cv::Vec3d& p, std::vector<int>& nvidxs, std::vector<float>* sqdis=nullptr) const;

    /**
     * Given a vertex ID returned from one of the find functions, return the actual position.
     * This is the position passed through the currently set transformation matrix.
     */
    cv::Vec3f pos( int) const;

    /**
     * Set the matrix through which all vertex lookups are assumed transformed by.
     * When calling any of the find functions, the point is first transformed through
     * the inverse of the matrix given here (the inverse of the matrix is actually stored).
     * The idea is that the transform matrix on the ObjModel that this ObjModelKDTree
     * was created from can be updated at the same time as the matrix is updated here,
     * and vertex lookups on will still return the correct (transformed) vertex IDs.
     */
    void setTransformMatrix( const cv::Matx44d&);

    /**
     * Add the given matrix to the existing one.
     */
    void addTransformMatrix( const cv::Matx44d& U) { setTransformMatrix( U * _tmat);}

    /**
     * Return the transform matrix set on this object (defaults to I).
     */
    const cv::Matx44d& transformMatrix() const { return _tmat;}

private:
    class Impl;
    Impl *_impl;
    cv::Matx44d _tmat, _imat;

    explicit ObjModelKDTree( const ObjModel&);
    ~ObjModelKDTree();
    ObjModelKDTree( const ObjModelKDTree&) = delete;
    ObjModelKDTree& operator=( const ObjModelKDTree&) = delete;

};  // end class


template <typename T>
class ObjModelPoints
{
public:
    ObjModelPoints( const ObjModel& m) : _m(m) {}
    inline size_t kdtree_get_point_count() const { return _m.numVtxs();}
    inline T kdtree_get_pt( const size_t idx, int dim) const { return _m.uvtx(idx)[dim];}
    template <class BBOX>
    inline bool kdtree_get_bbox( BBOX&) const { return false;}
private:
    const ObjModel& _m;
};  // end class

}   // end namespace

#endif
