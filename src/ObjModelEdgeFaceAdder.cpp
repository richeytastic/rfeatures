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

#include <ObjModelEdgeFaceAdder.h>
using RFeatures::ObjModelEdgeFaceAdder;
using RFeatures::ObjModel;
using RFeatures::ObjPoly;
#include <iostream>
#include <cassert>


// public
void ObjModelEdgeFaceAdder::addFaces( const std::unordered_map<int,IntSet>& xyset)
{
    _edgeUse.clear();
    typedef std::pair<int,IntSet> XYMap;
    for ( const XYMap& xys : xyset)
    {
        const int X = xys.first;
        const IntSet& yset = xys.second;   // Vertices connected to X
        for ( int Y : yset)
        {
            if ( xyset.count(Y))
            {
                const IntSet& zset = xyset.at(Y);
                for ( int Z : zset)
                {
                    if ( yset.count(Z))
                        setFace(X,Y,Z);
                }   // end foreach
            }   // end if
        }   // end foreach
    }   // end for
}   // end addFaces


// private
bool ObjModelEdgeFaceAdder::setFace( int x, int y, int z)
{
    init(x,y);
    init(x,z);
    init(y,z);
    const int xy = _edgeUse[x][y];
    const int xz = _edgeUse[x][z];
    const int yz = _edgeUse[y][z];

    // Reject faces that cannot be added because an edge is already shared by 2 faces.
    if ( xy == 2 || xz == 2 || yz == 2)
        return false;

    const int edgeSum = xy + xz + yz;

    // If only a single edge of the proposed triangle is shared by another face, just add the new triangle.
    // Alternatively, add triangle if no triangles already shared by the given edges themselves share a common edge.
    if ( edgeSum <= 1 || ( edgeSum >= 2 && !areSharedFacesJoined( x, y, z)))
    {
        addTriangle( x, y, z);
        return true;
    }   // end if

    return false;
}   // end setFace


// private
bool ObjModelEdgeFaceAdder::areSharedFacesJoined( int x, int y, int z) const
{
    int fxy = -1;
    if ( _mod->hasEdge(x,y))
        fxy = *_mod->spolys( x,y).begin();

    int fxz = -1;
    if ( _mod->hasEdge(x,z))
        fxz = *_mod->spolys( x,z).begin();

    int fyz = -1;
    if ( _mod->hasEdge(y,z))
        fyz = *_mod->spolys( y,z).begin();

    const ObjPoly* pxy = fxy >= 0 ? &_mod->face(fxy) : nullptr;
    const ObjPoly* pyz = fyz >= 0 ? &_mod->face(fyz) : nullptr;
    const ObjPoly* pxz = fxz >= 0 ? &_mod->face(fxz) : nullptr;

    return (pxy && pxz && pxy->isAdjacent( *pxz))
        || (pxz && pyz && pxz->isAdjacent( *pyz))
        || (pyz && pxy && pyz->isAdjacent( *pxy));
}   // end areSharedFacesJoined


// private
void ObjModelEdgeFaceAdder::init( int x, int y)
{
    if ( !_edgeUse.count(x) || !_edgeUse[x].count(y))
        _edgeUse[x][y] = 0;
}   // end init


// private
void ObjModelEdgeFaceAdder::addTriangle( int x, int y, int z)
{
    assert( _mod->face( x, y, z) == -1);
    _mod->addFace( x,y,z);
    _edgeUse[x][y]++;
    _edgeUse[x][z]++;
    _edgeUse[y][z]++;
}   // end addTriangle
