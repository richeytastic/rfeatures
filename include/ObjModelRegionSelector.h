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

#ifndef RFEATURES_OBJ_MODEL_REGION_SELECTOR_H
#define RFEATURES_OBJ_MODEL_REGION_SELECTOR_H

#include "ObjModel.h"

namespace RFeatures {

class rFeatures_EXPORT ObjModelRegionSelector
{
public:
    // Set the source vertex from which Euclidean distances are measured, and a seed vertex on the model from which
    // the selected component will be grown or shrunk as a single connected component. Initially, the selected component
    // will be the whole model (or at least that component connected to seedVtx). The initial component is parsed
    // across all edges connecting triangles and across all 1 dimensional vertices - i.e. vertices common to two
    // or more triangles not otherwise connected via an edge. This can be problematic for some algorithms that
    // require a triangulated manifold where all polygons connect via edges (so that triangle normals can be
    // propagated). If this is the case, ensure that the model supplied to this constructor first has none
    // of these kinds of vertices so that the component is always constructed from triangles sharing edges.
    // After construction, use adjustRadius to grow or shrink the model.
    typedef boost::shared_ptr<ObjModelRegionSelector> Ptr;
    static Ptr create( const ObjModel::Ptr, const cv::Vec3f& origin, int seedVtx=0);

    // Adjust the radius of the selected region to grow or shrink in size maintaining the old centre.
    // Returns the number of vertices within the newly selected region.
    size_t adjustRadius( double newRadiusThreshold);

    // Adjust the centre of the radial region to newPos maintaining the old radius value.
    // Returns the number of vertices within the newly selected region.
    size_t adjustPosition( const cv::Vec3f& newPos);

    // Get the boundary vertices.
    const IntSet* getBoundary() const { return _front;}

    // Sets the provided set to the face (polygon) indices of the input model that are within the selected region.
    void getRegionFaces( IntSet& cfids) const;

private:
    const ObjModel::Ptr _model;
    cv::Vec3f _ov;
    IntSet* _front;
    double _rad;
    IntSet _body;
    bool _trav1D;

    ObjModelRegionSelector( const ObjModel::Ptr, const cv::Vec3f& origin, int seedVtx=0);
    virtual ~ObjModelRegionSelector();
    ObjModelRegionSelector( const ObjModelRegionSelector&);   // No copy
    void operator=( const ObjModelRegionSelector&);    // No copy
};  // end class

}   // end namespace

#endif
