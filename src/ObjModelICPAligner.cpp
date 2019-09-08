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

#include <ObjModelICPAligner.h>
#include <opencv2/surface_matching/icp.hpp>
#include <cstring>
using RFeatures::ObjModelICPAligner;
using RFeatures::ObjModelCurvatureMap;
using RFeatures::ObjModel;


namespace {
cv::Mat_<float> createICPMatrix( const ObjModel& m, const ObjModelCurvatureMap& cm)
{
    const int n = m.numVtxs();
    cv::Mat_<float> tgt(n, 6);
    const IntSet& vidxs = m.vtxIds();
    assert(n == static_cast<int>(vidxs.size()));
    int i = 0;
    for ( int vid : vidxs)
    {
        // Copy the vertex into the first three positions
        const cv::Vec3f& v = m.vtx(vid);
        float* rowptr = tgt.ptr<float>(i++);
        memcpy( rowptr, &v[0], sizeof(float)*3);
        cv::Vec3f vnrm = cm.calcWeightedVertexNormal( m, vid);
        memcpy( &rowptr[3], &vnrm[0], sizeof(float)*3);
    }   // end for
    return tgt;
}   // end createICPMatrix
}   // end namespace


ObjModelICPAligner::ObjModelICPAligner( const ObjModel& m, const ObjModelCurvatureMap& cm) : _tgt( createICPMatrix(m, cm))
{
}   // end ctor


cv::Matx44d ObjModelICPAligner::calcTransform( const ObjModel& m, const ObjModelCurvatureMap& cm) const
{
    cv::Mat_<float> src = createICPMatrix( m, cm);
    double residual;
    double pose[16];
    cv::ppf_match_3d::ICP icp;
    icp.registerModelToScene( src, _tgt, residual, pose);
    return cv::Matx44d(pose);
}   // end calcTransform

