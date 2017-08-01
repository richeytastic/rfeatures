#include "ObjModelCopier.h"
using RFeatures::ObjModelCopier;
using RFeatures::ObjModelMover;
using RFeatures::ObjModel;
using RFeatures::ObjPoly;
#include <boost/foreach.hpp>
#include <boost/unordered_map.hpp>
#include <cmath>
#include <cassert>


// public
ObjModelCopier::ObjModelCopier( const ObjModel::Ptr src, const ObjModelMover& mover)
    : _model(src), _mover(mover)
{
    reset();
}   // end ctor


// public
void ObjModelCopier::reset()
{
    _cmodel = ObjModel::create( _model->getSpatialPrecision());
    // Copy in all the material data
    const int nmats = (int)_model->getNumMaterials();
    for ( int i = 0; i < nmats; ++i)
    {
        _cmodel->addMaterial();
        // Copy references to material texture maps
        const ObjModel::Material& m = _model->getMaterial(i);
        BOOST_FOREACH ( const cv::Mat& img, m.ambient)
            _cmodel->addMaterialAmbient( i, img);
        BOOST_FOREACH ( const cv::Mat& img, m.diffuse)
            _cmodel->addMaterialDiffuse( i, img);
        BOOST_FOREACH ( const cv::Mat& img, m.specular)
            _cmodel->addMaterialSpecular( i, img);
    }   // end for
}   // end reset


// protected
void ObjModelCopier::parseTriangle( int fid, int uvroot, int uva, int uvb)
{
    const int materialId = _model->getFaceMaterialId(fid);  // Will be -1 if no material for this face
    const ObjPoly& face = _model->getFace(fid); // Faces with original vertex IDs

    cv::Vec3f va = _model->getVertex( face.vindices[0]);
    cv::Vec3f vb = _model->getVertex( face.vindices[1]);
    cv::Vec3f vc = _model->getVertex( face.vindices[2]);

    _mover( va);
    _mover( vb);
    _mover( vc);

    const int n0 = _cmodel->addVertex( va);
    const int n1 = _cmodel->addVertex( vb);
    const int n2 = _cmodel->addVertex( vc);
    const int nfid = _cmodel->setFace( n0, n1, n2);

    if ( materialId >= 0)
    {
        boost::unordered_map<int,int> oldToNew;
        oldToNew[face.vindices[0]] = n0;
        oldToNew[face.vindices[1]] = n1;
        oldToNew[face.vindices[2]] = n2;

        const ObjModel::Material& material = _model->getMaterial( materialId);

        const cv::Vec3i& vorder = material.faceVertexOrder.at(fid);
        const cv::Vec6f& txs = material.txOffsets.at(fid);
        _cmodel->setFaceTextureOffsets( materialId, nfid, oldToNew.at(vorder[0]), cv::Vec2f( txs[0], txs[1]),
                                                          oldToNew.at(vorder[1]), cv::Vec2f( txs[2], txs[3]),
                                                          oldToNew.at(vorder[2]), cv::Vec2f( txs[4], txs[5]));
    }   // end if
}   // end parseTriangle
