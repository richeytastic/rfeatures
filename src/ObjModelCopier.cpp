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
using RFeatures::ObjModel;


// public
ObjModelCopier::ObjModelCopier( const ObjModel& source)
    : _model(source)
{
    _cmodel = ObjModel::create();
    _cmodel->copyInMaterials( source, true);    // Copy in all the material data
    _cmodel->setTransformMatrix( source.transformMatrix());
}   // end ctor


void ObjModelCopier::add( int fid)
{
    const int* vids = _model.fvidxs(fid);

    // Get the raw (untransformed) vertices.
    const cv::Vec3f& va = _model.uvtx( vids[0]);
    const cv::Vec3f& vb = _model.uvtx( vids[1]);
    const cv::Vec3f& vc = _model.uvtx( vids[2]);

    const int v0 = _cmodel->addVertex( va);
    const int v1 = _cmodel->addVertex( vb);
    const int v2 = _cmodel->addVertex( vc);
    const int nfid = _cmodel->addFace( v0, v1, v2);

    const int mid = _model.faceMaterialId(fid);  // Will be -1 if no material for this face
    if ( mid >= 0)
    {
        const int* uvids = _model.faceUVs(fid);
        _cmodel->setOrderedFaceUVs( mid, nfid, _model.uv(mid, uvids[0]),
                                               _model.uv(mid, uvids[1]),
                                               _model.uv(mid, uvids[2]));
    }   // end if
}   // end add
