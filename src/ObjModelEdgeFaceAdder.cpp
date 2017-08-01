#include <ObjModelEdgeFaceAdder.h>
using RFeatures::ObjModelEdgeFaceAdder;
using RFeatures::ObjModel;
using RFeatures::ObjPoly;
#include <cassert>
#include <iostream>
#include <boost/foreach.hpp>


// public
void ObjModelEdgeFaceAdder::addFaces( const boost::unordered_map<int,IntSet>& xyset)
{
    _edgeUse.clear();
    typedef std::pair<int,IntSet> XYMap;
    BOOST_FOREACH ( const XYMap& xys, xyset)
    {
        const int X = xys.first;
        const IntSet& yset = xys.second;   // Vertices connected to X
        BOOST_FOREACH ( int Y, yset)
        {
            //std::cerr << "   [" << std::right << std::setw(2) << X << "]-->[" << std::right << std::setw(2) << Y << "]" << std::endl;
            if ( xyset.count(Y))
            {
                const IntSet& zset = xyset.at(Y);
                BOOST_FOREACH ( int Z, zset)
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
    {
        //std::cerr << "    X Invalid [" << x << "],[" << y << "],[" << z << "]" << std::endl;
        return false;
    }   // end if

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
        fxy = *_mod->getSharedFaces( x,y).begin();

    int fxz = -1;
    if ( _mod->hasEdge(x,z))
        fxz = *_mod->getSharedFaces( x,z).begin();

    int fyz = -1;
    if ( _mod->hasEdge(y,z))
        fyz = *_mod->getSharedFaces( y,z).begin();

    return sharesCommonEdge( fxy, fxz) || sharesCommonEdge( fxz, fyz) || sharesCommonEdge( fyz, fxy);
}   // end areSharedFacesJoined


// private
bool ObjModelEdgeFaceAdder::sharesCommonEdge( int f0, int f1) const
{
    if ( f0 == -1 || f1 == -1)
        return false;

    const ObjPoly& face = _mod->getFace(f0);
    return _mod->getSharedFaces( face.vindices[0], face.vindices[1]).count(f1) ||
           _mod->getSharedFaces( face.vindices[1], face.vindices[2]).count(f1) ||
           _mod->getSharedFaces( face.vindices[2], face.vindices[0]).count(f1);
}   // end sharesCommonEdge


// private
void ObjModelEdgeFaceAdder::init( int x, int y)
{
    if ( !_edgeUse.count(x) || !_edgeUse[x].count(y))
        _edgeUse[x][y] = 0;
}   // end init


// private
void ObjModelEdgeFaceAdder::addTriangle( int x, int y, int z)
{
    const int nfid = _mod->getFaceId( x, y, z);
    assert( nfid == -1);
    _mod->setFace( x,y,z);
    //std::cerr << "    + Set face   ["<< x <<"],["<< y << "],[" << z << "]" << std::endl;
    _edgeUse[x][y]++;
    _edgeUse[x][z]++;
    _edgeUse[y][z]++;
}   // end addTriangle
