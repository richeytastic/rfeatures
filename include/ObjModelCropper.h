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

#ifndef RFEATURES_OBJ_MODEL_CROPPER_H
#define RFEATURES_OBJ_MODEL_CROPPER_H

#include "ObjModelTriangleMeshParser.h"

namespace RFeatures
{

class rFeatures_EXPORT ObjModelCropper : public RFeatures::ObjModelBoundaryParser
{
public:
    ObjModelCropper( const ObjModel::Ptr m, const cv::Vec3f& originVertex, double radiusThreshold);

protected:
    virtual bool parseEdge( int fid, int vid0, int vid1);

private:
    const ObjModel::Ptr _m;
    const cv::Vec3f _ov;
    double _sqRadiusThreshold;
};  // end class


}   // end namespace

#endif
