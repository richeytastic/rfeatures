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
        const int vidxs[3] = {oldToNew[vorder[0]], oldToNew[vorder[1]], oldToNew[vorder[2]]};
        _cmodel->setOrderedFaceTextureOffsets( materialId, nfid, vidxs, (const cv::Vec2f*)(&txs));
    }   // end if
}   // end parseTriangle
