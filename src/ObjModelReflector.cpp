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


void ObjModelReflector::reflectPoint( cv::Vec3f& v, const cv::Vec3f& pt, const cv::Vec3f& pvec)
{
    const cv::Vec3f dvec = pt - v;  // Difference vector to point on the plane
    const float d = pvec.dot( dvec);// Project to plane vector to give orthogonal distance to plane
    v = v + 2.0f*d*pvec;            // Reflect point through plane
}   // end reflectPoint
                                                        

void ObjModelReflector::reflect( const cv::Vec3f& pt, const cv::Vec3f& plane)
{
    cv::Vec3f pvec; // Ensure plane vector is normalized
    cv::normalize( plane, pvec);

    const IntSet& vids = _model->vtxIds();
    for ( int vidx : vids)
    {
        cv::Vec3f v = _model->vtx(vidx);    // Copy out
        reflectPoint( v, pt, pvec);
        _model->adjustVertex( vidx, v[0], v[1], v[2]);
    }   // end for

    // Also need to flip normals on all the model's faces
    const IntSet& fids = _model->faces();
    for ( int fid : fids)
        _model->reversePolyVertices(fid);
}   // end reflect
