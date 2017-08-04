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

#ifndef RFEATURES_OBJ_MODEL_EDGE_FACE_ADDER_H
#define RFEATURES_OBJ_MODEL_EDGE_FACE_ADDER_H

#include "ObjModel.h"

namespace RFeatures
{

class ObjModelEdgeFaceAdder
{
public:
    explicit ObjModelEdgeFaceAdder( ObjModel::Ptr mod) : _mod(mod) {}

    // The vertex IDs in xyset must already be present in the model.
    void addFaces( const boost::unordered_map<int,IntSet>& xyset);

private:
    ObjModel::Ptr _mod;
    boost::unordered_map<int, boost::unordered_map<int, int> > _edgeUse; // Count number of times each edge is used

    bool setFace( int x, int y, int z);
    bool areSharedFacesJoined( int x, int y, int z) const;
    bool sharesCommonEdge( int f0, int f1) const;
    void init( int x, int y);
    void addTriangle( int x, int y, int z);
};  // end class

}   // end namespace

#endif
