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

#include <ObjModelReflector.h>
#include <algorithm>
using RFeatures::ObjModelReflector;
using RFeatures::ObjModel;

// public
ObjModelReflector::ObjModelReflector( ObjModel::Ptr model) : _model(model) {}
                                                        

void ObjModelReflector::reflect( const cv::Vec3f& pt, const cv::Vec3f& plane)
{
    cv::Vec3f pvec; // Ensure plane vector is normalized
    cv::normalize( plane, pvec);

    for ( int vidx : _model->getVertexIds())
    {
        const cv::Vec3f& v = _model->vtx(vidx);

        const cv::Vec3f dvec = pt - v;      // Difference vector to point on the plane
        const float d = pvec.dot( dvec);    // Project to plane vector to give orthogonal distance to plane
        const cv::Vec3f nv = v + 2.0f*d*pvec;  // Reflect point through plane
        _model->adjustVertex( vidx, nv[0], nv[1], nv[2]);
    }   // end for

    // Also need to flip normals on all the model's faces
    const IntSet& fids = _model->getFaceIds();
    std::for_each( std::begin(fids), std::end(fids), [&](int fid){ _model->reverseFaceVertices( fid);});
}   // end reflect
