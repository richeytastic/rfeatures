#include "ObjModelNormalCalculator.h"
using RFeatures::ObjModelNormalCalculator;
using RFeatures::ObjModel;
using RFeatures::ObjPoly;
#include <boost/foreach.hpp>
#include <cassert>

// public
ObjModelNormalCalculator::ObjModelNormalCalculator( const ObjModel::Ptr m)
    : _model(m)
{}   // end ctor


// public
void ObjModelNormalCalculator::reset()
{
    _faceNormals.clear();
    _faceVtxOrder.clear();
}   // end reset


// public
cv::Vec3d ObjModelNormalCalculator::operator()( int root, int a, int b) const
{
    const cv::Vec3f& vroot = _model->vtx( root);
    const cv::Vec3f& va = _model->vtx( a);
    const cv::Vec3f& vb = _model->vtx( b);
    const cv::Vec3d vadelta = va - vroot;
    const cv::Vec3d vbdelta = vb - vroot;
    cv::Vec3d nrm;
    cv::normalize( vadelta.cross(vbdelta), nrm);
    return nrm;
}   // end operator()


int getAdjacentFace( const ObjModel::Ptr model, int fid)
{
    const ObjPoly& face = model->getFace(fid);
    const IntSet& sfids0 = model->getSharedFaces(face.vindices[0], face.vindices[1]);
    const IntSet& sfids1 = model->getSharedFaces(face.vindices[1], face.vindices[2]);
    const IntSet& sfids2 = model->getSharedFaces(face.vindices[0], face.vindices[2]);
    const IntSet* sfidsPtrs[3] = { &sfids0, &sfids1, &sfids2};

    int tfid = fid;
    for ( int i = 0; i < 3; ++i)
    {
        const IntSet& sfids = *sfidsPtrs[i];
        if ( sfids.size() == 1)
            continue;

        tfid = *sfids.begin();
        if ( tfid == fid)   // Only two faces since triangulation
        {
            tfid = *(++sfids.begin());
            assert( tfid != fid);
            break;
        }   // end if
    }   // end for

    return tfid;
}   // end getAdjacentFace


int notin( const IntSet& iset, int k, int j)
{
    BOOST_FOREACH ( int i, iset)
        if ( i != k && i != j)
            return i;
}   // end notin

// public
const cv::Vec3d& ObjModelNormalCalculator::recalcFaceNormal( int fid)
{
    // If this polygon does not already exist, need to find its neighbours
    // and determine the vertex ordering based on these.
    if ( !_faceVtxOrder.count(fid))
    {
        // Use the vertex ordering on the adjacent face to determine the correct vertex ordering for fid.
        const int afid = getAdjacentFace( _model, fid);
        assert( _faceVtxOrder.count(afid) > 0);

        const ObjPoly& face = _model->getFace(fid);
        IntSet fvindices;   // Create a set for easy testing of vertex sharing
        fvindices.insert( &face.vindices[0], &face.vindices[2]);

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
    return _faceNormals[fid] = operator()( vorder[0], vorder[1], vorder[2]);
}   // end recalcFaceNormal


// public
void ObjModelNormalCalculator::remove( int fid)
{
    _faceNormals.erase(fid);
    _faceVtxOrder.erase(fid);
}   // end remove


// protected virtual
void ObjModelNormalCalculator::parseTriangle( int fid, int root, int a, int b)
{
    _faceNormals[fid] = operator()( root, a, b);
    _faceVtxOrder[fid] = cv::Vec3i( root, a, b);
}   // end parseTriangle
