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

#ifndef RFEATURES_OBJ_MODEL_MESH_TRAVERSAL_RECORDER_H
#define RFEATURES_OBJ_MODEL_MESH_TRAVERSAL_RECORDER_H

#include "ObjModelTriangleMeshParser.h"

namespace RFeatures
{

class rFeatures_EXPORT ObjModelMeshTraversalRecorder : public ObjModelTriangleParser
{
public:
    ObjModelMeshTraversalRecorder();

    virtual void reset();
    const IntSet& getTraversedVertices() const { return _vidxs;}

protected:
    virtual void parseTriangle( int fid, int uvroot, int uva, int uvb);

private:
    IntSet _vidxs;
    ObjModelMeshTraversalRecorder( const ObjModelMeshTraversalRecorder&);   // No copy
    void operator=( const ObjModelMeshTraversalRecorder&); // No copy
};  // end class

}   // end namespace

#endif
