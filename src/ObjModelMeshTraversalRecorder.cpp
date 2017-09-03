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

#include <ObjModelMeshTraversalRecorder.h>
using RFeatures::ObjModelMeshTraversalRecorder;
using RFeatures::ObjModelTriangleParser;


// public
ObjModelMeshTraversalRecorder::ObjModelMeshTraversalRecorder() {}

// public
void ObjModelMeshTraversalRecorder::reset()
{
    _vidxs.clear();
}   // end reset

// protected
void ObjModelMeshTraversalRecorder::parseTriangle( int f, int r, int a, int b)
{
    _vidxs.insert(r);
    _vidxs.insert(a);
    _vidxs.insert(b);
}   // end parseTriangle
