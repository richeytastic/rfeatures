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
using RFeatures::Transformer;
using RFeatures::ObjModel;
#include <algorithm>
#include <cassert>
#include <cmath>


// public
ObjModelCopier::ObjModelCopier( const ObjModel* source, const Transformer* mover)
    : _model(source), _mover(mover)
{
    assert( _model != NULL);
    _cmodel = ObjModel::create( _model->getSpatialPrecision());
    _oldToNewMat.clear();

    // Copy in all the material data
    const IntSet& matIds = _model->getMaterialIds();
    for ( int matId : matIds)
    {
        const int newMatId = _cmodel->addMaterial();
        _oldToNewMat[matId] = newMatId; // Map reference to new material in copied model

        // Copy references to material texture maps
        for ( const cv::Mat& img : _model->getMaterialAmbient(matId))
            _cmodel->addMaterialAmbient( newMatId, img);
        for ( const cv::Mat& img : _model->getMaterialDiffuse(matId))
            _cmodel->addMaterialDiffuse( newMatId, img);
        for ( const cv::Mat& img : _model->getMaterialSpecular(matId))
            _cmodel->addMaterialSpecular( newMatId, img);
    }   // end foreach
}   // end reset


// public
void ObjModelCopier::addTriangle( int fid)
{
    assert( _cmodel != NULL);
    const int materialId = _model->getFaceMaterialId(fid);  // Will be -1 if no material for this face
    const int* vids = _model->getFaceVertices(fid); // Original vertex IDs

    const cv::Vec3f& va = _model->vtx( vids[0]);
    const cv::Vec3f& vb = _model->vtx( vids[1]);
    const cv::Vec3f& vc = _model->vtx( vids[2]);

    int v0, v1, v2;
    if ( _mover)
    {
        v0 = _cmodel->addVertex( _mover->transform(va));
        v1 = _cmodel->addVertex( _mover->transform(vb));
        v2 = _cmodel->addVertex( _mover->transform(vc));
    }   // end if
    else
    {
        v0 = _cmodel->addVertex( va);
        v1 = _cmodel->addVertex( vb);
        v2 = _cmodel->addVertex( vc);
    }   // end else

    const int nfid = _cmodel->addFace( v0, v1, v2);

    if ( materialId >= 0)
    {
        const int* uvids = _model->getFaceUVs(fid);
        const int newMatId = _oldToNewMat.at(materialId);
        _cmodel->setOrderedFaceUVs( newMatId, nfid, _model->uv(materialId, uvids[0]),
                                                    _model->uv(materialId, uvids[1]),
                                                    _model->uv(materialId, uvids[2]));
    }   // end if
}   // end addTriangle
