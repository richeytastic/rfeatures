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

#ifndef RFEATURES_OBJ_MODEL_POLY_UNFOLDER_H
#define RFEATURES_OBJ_MODEL_POLY_UNFOLDER_H

#include "ObjModel.h"

namespace RFeatures {

class rFeatures_EXPORT ObjModelPolyUnfolder
{
public:
    explicit ObjModelPolyUnfolder( const ObjModel*);
    ObjModelPolyUnfolder( const ObjModel*, int T);  // Also resets to T

    // Make a new unfolder, but using the ALREADY UNFOLDED polygon T from the argument unfolder.
    // This means that the new unfolder will be unfolding into the same plane as the argument
    // unfolder, but with T as the sole unfolded polygon.
    ObjModelPolyUnfolder( const ObjModelPolyUnfolder&, int T);

    inline const ObjModel* model() const { return _model;}

    // Reset the unfolded set to start unfolding on the plane incident with poly T.
    void reset( int T=-1);

    // Assumes that the given vertex has already been unfolded!
    inline const cv::Vec3d& uvtx( int vidx) const { return _uvtxs.at(vidx);}
    inline bool isVertexUnfolded( int vidx) const { return _uvtxs.count(vidx) > 0;}
    inline bool isPolyUnfolded( int fid) const { return _upolys.count(fid) > 0;}
    inline const IntSet& upolys() const { return _upolys;}

    // Given point p with respect to the ORIGINAL (folded) polygon T, calculate and return the
    // point's position with respect to the unfolded T. Assumes that T is already unfolded!
    cv::Vec3d calcUnfoldedPoint( int T, const cv::Vec3f& p) const;

    // Unfold triangle T along vertex edge r,a, causing vertex b (opposite to r,a) on
    // triangle T to be moved into the unfolding plane. If r,a is already unfolded, T
    // is unfolded along r,a to be in the same plane as that polygon. If r,a is not already
    // in the set of unfolded vertices, the set of unfolded triangles is reset to begin with T.
    // It is not necessary to give r,a in any special order.
    // Returns b. Use unfoldedVertex to get its unfolded position.
    // Safe to call multiple times with same T.
    // Returns -1 if T is not a valid polygon ID.
    int unfold( int T, int r, int a);

    // Return normal defining the plane being unfolded into (zero if nothing yet unfolded).
    const cv::Vec3d& norm() const { return _pnorm;}

    // Unfold a string of polygons adjacent to sequential path of vertices vidxs
    // starting at polygon sT and ending at polygon fT. Reset first.
    void unfoldPath( const std::vector<int>& vidxs, int sT, int fT);

private:
    const ObjModel *_model;
    cv::Vec3d _pnorm;
    std::unordered_map<int, cv::Vec3d> _uvtxs;  // Positions of "unfolded" vertices
    IntSet _upolys;  // Ids of unfolded polygons (including constructor poly).
};  // end class

}   // end namespace

#endif
