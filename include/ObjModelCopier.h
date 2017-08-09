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

#ifndef RFEATURES_OBJ_MODEL_COPIER_H
#define RFEATURES_OBJ_MODEL_COPIER_H

/**
 * Copy a source model only from the parsed triangles with optional moving of vertices.
 */

#include "ObjModelTriangleMeshParser.h"
#include "ObjModelMover.h"

namespace RFeatures
{

class rFeatures_EXPORT ObjModelCopier : public ObjModelTriangleParser
{
public:
    ObjModelCopier( const ObjModel::Ptr source, const ObjModelMover* mover=NULL);

    void setRotationMatrix( double radians, const cv::Vec3d& axis);
    void setRotationMatrix( const cv::Matx33d& rotMat);

    void reset();

    ObjModel::Ptr getCopiedModel() const { return _cmodel;}

protected:
    virtual void parseTriangle( int fid, int uvroot, int uva, int uvb);

private:
    const ObjModel::Ptr _model;
    const ObjModelMover* _mover;
    ObjModel::Ptr _cmodel;
};  // end class

}   // end namespace

#endif
