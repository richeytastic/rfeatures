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

#include <ObjModelReferenceResampler.h>
#include <ObjModelTools.h>
#include <iostream>
#include <cassert>
#include <string>
#include <cmath>
using RFeatures::ObjModelReferenceResampler;
using RFeatures::ObjModelKDTree;
using RFeatures::ObjModel;


ObjModelReferenceResampler::ObjModelReferenceResampler( const ObjModel* tgt, const ObjModel* tlmks, int k)
    : _tgt(tgt), _k( std::max(1, std::min( int(tlmks->numVtxs()) / 3, k)))
{
    assert( tgt->hasSequentialVertexIds());
    assert( tlmks->hasSequentialVertexIds());
    assert( tlmks->numVtxs() >= 3);
    _tlmks = ObjModelKDTree::create( tlmks);
    if ( k > int(tlmks->numVtxs()) / 3)
        std::cerr << "RFeatures::ObjModelReferenceResampler: k set to not exceed number of available landmarks!" << std::endl;
}   // end ctor


ObjModel::Ptr ObjModelReferenceResampler::sample( const ObjModelKDTree* kdt, const ObjModel* slmks) const
{
    static const std::string errhd = "[ERROR] RFeatures::ObjModelReferenceResampler::sample: ";

    const ObjModel* smod = kdt->model();
    assert(smod->hasSequentialVertexIds());
    const int nlmks = int(_tlmks->model()->numVtxs());
    if ( int(slmks->numVtxs()) != nlmks)
    {
        std::cerr << errhd << "Target and source reference landmark count mismatch!" << std::endl;
        return nullptr;
    }   // end if

    static const double EPS = 0.0000001;

    const int K = _k;
    std::vector<int> kverts(3*K);

    ObjModel::Ptr nmod = ObjModel::create();

    Triangle3f tplane, splane;  // The triangle planes defined by triples of landmark points.
    cv::Vec3f u;

    const int n = int(_tgt->numVtxs());
    for ( int vtxId = 0; vtxId < n; ++vtxId)
    {
        const cv::Vec3f& v = _tgt->vtx(vtxId);
        _tlmks->findn( v, kverts);  // Find the kverts.size() closest landmarks to v.

        // The mean of the estimated points on the source model is found, with the contribution of
        // each estimated point to the mean scaled as the inverse of its squared distance from the
        // reference plane given by the corresponding triple of landmarks.
        cv::Vec3d mpos(0,0,0);
        double denom = 0;
        for ( size_t i = 0; i < kverts.size(); i += 3)
        {
            // The indices of the three landmarks forming a reference plane
            int lmidx0 = kverts.at(i+0);
            int lmidx1 = kverts.at(i+1);
            int lmidx2 = kverts.at(i+2);

            // Get the reference landmarks for the target mesh.
            tplane[0] = _tlmks->model()->vtx( lmidx0);
            tplane[1] = _tlmks->model()->vtx( lmidx1);
            tplane[2] = _tlmks->model()->vtx( lmidx2);

            // Get the coregistered landmarks for the source mesh (same indices).
            splane[0] = slmks->vtx( lmidx0);
            splane[1] = slmks->vtx( lmidx1);
            splane[2] = slmks->vtx( lmidx2);

            // Find u as the estimated mapped position of v to the source mesh.
            // Returned value d is the distance of u from the reference triple of landmarks.
            const double d = mapPosition( tplane, v, splane, u);
            const double a = pow( EPS + d*d, -1); // Affinity weight as inverse distance
            mpos[0] += a*u[0];
            mpos[1] += a*u[1];
            mpos[2] += a*u[2];
            denom += a;
        }   // end for

        // mpos is the best estimate of a coregistered point near the surface of the source model.
        nmod->addVertex( float(mpos[0]/denom), float(mpos[1]/denom), float(mpos[2]/denom));
    }   // end for

    // Finally, set the connectivity between vertices on nmod to match the target mesh.
    const IntSet& fids = _tgt->faces();
    for ( int fid : fids)
    {
        const ObjPoly& f = _tgt->face(fid);
        nmod->addFace( f[0], f[1], f[2]);
    }   // end for

    return nmod;
}   // end sample
