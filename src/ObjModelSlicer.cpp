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

#include <ObjModelSlicer.h>
#include <ObjModelCopier.h>
#include <ObjPolyPlane.h>
#include <FeatureUtils.h>
#include <cmath>
using RFeatures::ObjModelSlicer;
using RFeatures::ObjModel;


// public
ObjModelSlicer::ObjModelSlicer( const ObjModel* src) : _model(src) {}


ObjModel::Ptr ObjModelSlicer::operator()( const cv::Vec3f& p, const cv::Vec3f& vec) const
{
    cv::Vec3f n;    // Ensure the plane vector is normalised
    cv::normalize( vec, n);

    const ObjModel* mod = _model;
    ObjModelCopier copier( mod);
    ObjModel::Ptr hmod = copier.copiedModel();

    int mid;
    cv::Vec2f uvyb, uvyc;
    cv::Vec3f yb, yc;
    const IntSet& fids = _model->faces();
    for ( int fid : fids)
    {
        const ObjPolyPlane fp( mod, fid, p, n);
        const int nihs = fp.inhalf();

        if ( nihs == -1) // All face vertices in the wrong half so ignore
            continue;
        else if ( nihs == 1)
            copier.add(fid);    // All face vertices in right half so copy in normally
        else
        {
            fp.findPlaneVertices( yb, yc);
            const int y = hmod->addVertex( yb);
            const int z = hmod->addVertex( yc);

            mid = mod->faceMaterialId(fid);
            if ( mid >= 0)
            {
                uvyb = mod->calcTextureCoords( fid, yb);
                uvyc = mod->calcTextureCoords( fid, yc);
            }   // end if

            if ( fp.inside()) // Only a single vertex is in the half space so new triangle easy
            {
                const int x = hmod->addVertex( fp.va());   // In half space
                const int nfid = hmod->addFace( x, y, z);
                if ( mid >= 0)
                    hmod->setOrderedFaceUVs( mid, nfid, fp.uva(), uvyb, uvyc);
            }   // end if
            else    // Two vertices in the half space so need to add two new triangles
            {
                const cv::Vec3f& xb = fp.vb();   // In half space
                const cv::Vec3f& xc = fp.vc();   // In half space
                const int v = hmod->addVertex( xb);
                const int x = hmod->addVertex( xc);

                // What's the shortest diagonal to split the two new triangles? yb,xc or yc,xb?
                if ( cv::norm( yb-xc) < cv::norm( yc-xb))
                {
                    const int nfid0 = hmod->addFace( y, v, x);
                    const int nfid1 = hmod->addFace( x, z, y);
                    if ( mid >= 0)
                    {
                        const cv::Vec2f& uvb = fp.uvb();
                        const cv::Vec2f& uvc = fp.uvc();
                        hmod->setOrderedFaceUVs( mid, nfid0, uvyb, uvb, uvc);
                        hmod->setOrderedFaceUVs( mid, nfid1, uvc, uvyc, uvyb);
                    }   // end if
                }   // end if
                else
                {
                    const int nfid0 = hmod->addFace( v, x, z);
                    const int nfid1 = hmod->addFace( z, y, v);
                    if ( mid >= 0)
                    {
                        const cv::Vec2f& uvb = fp.uvb();
                        const cv::Vec2f& uvc = fp.uvc();
                        hmod->setOrderedFaceUVs( mid, nfid0, uvb, uvc, uvyc);
                        hmod->setOrderedFaceUVs( mid, nfid1, uvyc, uvyb, uvb);
                    }   // end if
                }   // end else
            }   // end else
        }   // end else
    }   // end for

    return hmod;
}   // end operator()
