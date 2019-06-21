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

#ifndef RFEATURES_OBJ_MODEL_SLICER_H
#define RFEATURES_OBJ_MODEL_SLICER_H

/**
 * Copy out one half of a source model along the defined plane creating new
 * vertices and faces where necessary to ensure that the triangles intersect
 * cleanly at the planar boundary.
 **/

#include "ObjModel.h"

namespace RFeatures {

class rFeatures_EXPORT ObjModelSlicer
{
public:
    // Provide the source model.
    ObjModelSlicer( const ObjModel& source);

    // Provide a plane defined by a point and a vector pointing into the half of the model wanted.
    ObjModel::Ptr operator()( const cv::Vec3f& pt, const cv::Vec3f& vec) const;

private:
    const ObjModel& _model;
    ObjModelSlicer( const ObjModelSlicer&) = delete;
    void operator=( const ObjModelSlicer&) = delete;
};  // end class

}   // end namespace

#endif
