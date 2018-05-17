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
    // Sets the triangle that represents the unfolding plane using the given model.
	// All of this triangle's vertices will be set in the unfolded set, and so
    // unfoldAlongEdge can be called with any pair of vertices from triangle T.
    ObjModelPolyUnfolder( const ObjModel*, int T);

    const ObjModel* model() const { return _model;}

    // Unfold triangle T along vertex edge v0,v1, causing vertex v2 to be moved into
    // the unfolding plane. Vertices v0 and v1 must already be in the set of unfolded
    // vertices. It is not necessary to give v0,v1 in any particular order; they will
    // be checked to ensure the edge vector formed by these two vertices points in the
    // correct direction for the plane being unfolded into. Returns the ID of the vertex
    // that has been newly rotated into the plane. Use getUnfoledVertex to get the
    // unfolded position of this vertex. Since this function records the positions of
    // the vertices it unfolds, it can be called with successive adjacent triangles
    // to unfold an entire region of a surface into a plane.
    int unfoldAlongEdge( int T, int v0, int v1);

    const cv::Vec3d& getUnfoldedVertex( int vidx) const { return _unfoldedUVs.at(vidx);}
    const cv::Vec3d& getUnfoldingPlane() const { return _planeNormal;}

private:
    const ObjModel* _model;

    std::unordered_map<int, cv::Vec3d> _unfoldedUVs;  // Positions of "unfolded" vertices
    cv::Vec3d _planeNormal;    // Normal definining the "unfolding" plane

    ObjModelPolyUnfolder( const ObjModelPolyUnfolder&); // No copy
    void operator=( const ObjModelPolyUnfolder&);       // No copy
};  // end class

}   // end namespace

#endif
