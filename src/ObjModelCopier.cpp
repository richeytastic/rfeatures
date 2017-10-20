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

#include <ObjModelCopier.h>
using RFeatures::ObjModelCopier;
using RFeatures::ObjModelMover;
using RFeatures::ObjModel;
using RFeatures::ObjPoly;
#include <boost/foreach.hpp>
#include <boost/unordered_map.hpp>
#include <cmath>
#include <cassert>


// public
ObjModelCopier::ObjModelCopier( const ObjModelMover* mover) : _mover(mover)
{
}   // end ctor


// public
ObjModelCopier::~ObjModelCopier(){}


// public
void ObjModelCopier::reset()
{
    assert( model != NULL);
    _cmodel = ObjModel::create( model->getSpatialPrecision());
    _oldToNewMat.clear();

    // Copy in all the material data
    const IntSet& matIds = model->getMaterialIds();
    BOOST_FOREACH ( int matId, matIds)
    {
        const int newMatId = _cmodel->addMaterial();
        _oldToNewMat[matId] = newMatId; // Map reference to new material in copied model

        // Copy references to material texture maps
        BOOST_FOREACH ( const cv::Mat& img, model->getMaterialAmbient(matId))
            _cmodel->addMaterialAmbient( newMatId, img);
        BOOST_FOREACH ( const cv::Mat& img, model->getMaterialDiffuse(matId))
            _cmodel->addMaterialDiffuse( newMatId, img);
        BOOST_FOREACH ( const cv::Mat& img, model->getMaterialSpecular(matId))
            _cmodel->addMaterialSpecular( newMatId, img);
    }   // end foreach
}   // end reset


// protected
void ObjModelCopier::parseTriangle( int fid, int uvroot, int uva, int uvb)
{
    assert( _cmodel != NULL);
    const int materialId = model->getFaceMaterialId(fid);  // Will be -1 if no material for this face
    const int* vids = model->getFaceVertices(fid); // Original vertex IDs

    const cv::Vec3f& va = model->vtx( vids[0]);
    const cv::Vec3f& vb = model->vtx( vids[1]);
    const cv::Vec3f& vc = model->vtx( vids[2]);

    int v0, v1, v2;
    if ( _mover)
    {
        v0 = _cmodel->addVertex( (*_mover)(va));
        v1 = _cmodel->addVertex( (*_mover)(vb));
        v2 = _cmodel->addVertex( (*_mover)(vc));
    }   // end if
    else
    {
        v0 = _cmodel->addVertex( va);
        v1 = _cmodel->addVertex( vb);
        v2 = _cmodel->addVertex( vc);
    }   // end else

    const int nfid = _cmodel->setFace( v0, v1, v2);

    if ( materialId >= 0)
    {
        const int* uvids = model->getFaceUVs(fid);
        const int newMatId = _oldToNewMat.at(materialId);
        _cmodel->setOrderedFaceUVs( newMatId, nfid, v0, model->uv(materialId, uvids[0]),
                                                    v1, model->uv(materialId, uvids[1]),
                                                    v2, model->uv(materialId, uvids[2]));
    }   // end if
}   // end parseTriangle
