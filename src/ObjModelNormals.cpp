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

#include <ObjModelNormals.h>
using RFeatures::ObjModelNormals;
using RFeatures::ObjModel;
using RFeatures::ObjPoly;
#include <cassert>


// public
ObjModelNormals::ObjModelNormals() {}   // end ctor


// public
void ObjModelNormals::reset()
{
    _faceNormals.clear();
    _faceVtxOrder.clear();
}   // end reset


// public static
cv::Vec3d ObjModelNormals::calcNormal( const ObjModel& model, int root, int a, int b)
{
    const cv::Vec3f& vroot = model.vtx( root);
    const cv::Vec3f& va = model.vtx( a);
    const cv::Vec3f& vb = model.vtx( b);
    const cv::Vec3d vadelta = va - vroot;
    const cv::Vec3d vbdelta = vb - vroot;
    cv::Vec3d nrm;
    cv::normalize( vadelta.cross(vbdelta), nrm);
    return nrm;
}   // end calcNormal


// public static
cv::Vec3d ObjModelNormals::calcNormal( const ObjModel& model, int fid)
{
    const int* vindices = model.fvidxs(fid);
    return calcNormal( model, vindices[0], vindices[1], vindices[2]);
}   // end calcNormal


namespace {
int getAdjacentFace( const ObjModel& model, int fid)
{
    const ObjPoly& f = model.face(fid);
    const IntSet& sfids0 = model.spolys(f[0], f[1]);
    const IntSet& sfids1 = model.spolys(f[1], f[2]);
    const IntSet& sfids2 = model.spolys(f[2], f[0]);
    const IntSet* sfidsPtrs[3] = { &sfids0, &sfids1, &sfids2};

    int tf = fid;
    for ( int i = 0; i < 3; ++i)
    {
        const IntSet& sfids = *sfidsPtrs[i];
        if ( sfids.size() == 1)
            continue;

        tf = *sfids.begin();
        if ( tf == fid)   // Only two faces since triangulation
        {
            tf = *(++sfids.begin());
            assert( tf != fid);
            break;
        }   // end if
    }   // end for

    return tf;
}   // end getAdjacentFace


int notin( const IntSet& iset, int k, int j)
{
    for ( int i : iset)
        if ( i != k && i != j)
            return i;
    assert(false);  // Should never reach!
    return -1;  // Silence compiler warnings
}   // end notin

}   // end namespace


// public
const cv::Vec3d& ObjModelNormals::recalcFaceNormal( int fid)
{
    // If this polygon does not already exist, need to find its neighbours
    // and determine the vertex ordering based on these.
    if ( !_faceVtxOrder.count(fid))
    {
        // Use the vertex ordering on the adjacent face to determine the correct vertex ordering for fid.
        const int afid = getAdjacentFace( *model, fid);
        assert( _faceVtxOrder.count(afid) > 0);

        const int* vids = model->fvidxs(fid);
        IntSet fvindices;   // Create a set for easy testing of vertex sharing
        fvindices.insert( &vids[0], &vids[2]);

        const cv::Vec3i& aorder = _faceVtxOrder.at(afid);   // The ordering on the adjacent face
        IntSet avindices;
        avindices.insert( &aorder[0], &aorder[2]);

        cv::Vec3i forder;
        if ( fvindices.count(aorder[0]) && fvindices.count(aorder[1]))
            forder = cv::Vec3i( aorder[1], aorder[0], notin( fvindices, aorder[1], aorder[0]));
        else if ( fvindices.count(aorder[0]) && fvindices.count(aorder[2]))
            forder = cv::Vec3i( aorder[0], aorder[2], notin( fvindices, aorder[0], aorder[2]));
        else if ( fvindices.count(aorder[1]) && fvindices.count(aorder[2]))
            forder = cv::Vec3i( aorder[2], aorder[1], notin( fvindices, aorder[2], aorder[1]));
        else
            assert(false);
        _faceVtxOrder[fid] = forder;
    }   // end if

    const cv::Vec3i& vorder = _faceVtxOrder.at(fid);
    return _faceNormals[fid] = calcNormal( *model, vorder[0], vorder[1], vorder[2]);
}   // end recalcFaceNormal


// public
void ObjModelNormals::remove( int fid)
{
    _faceNormals.erase(fid);
    _faceVtxOrder.erase(fid);
}   // end remove


// protected virtual
void ObjModelNormals::parseTriangle( int fid, int root, int a, int b)
{
    _faceNormals[fid] = calcNormal( *model, root, a, b);
    _faceVtxOrder[fid] = cv::Vec3i( root, a, b);
}   // end parseTriangle
