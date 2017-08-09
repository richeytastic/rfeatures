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

#include <ObjModelTetrahedronReplacer.h>
using RFeatures::ObjModelTetrahedronReplacer;
using RFeatures::ObjModel;
#include <boost/foreach.hpp>
#include <cassert>


ObjModelTetrahedronReplacer::ObjModelTetrahedronReplacer( ObjModel::Ptr m) : _model(m) {}


int ObjModelTetrahedronReplacer::removeTetrahedrons()
{
    int removedVertexCount = 0;
    int bfid;
    int vidxs[3];
    int fids[3];

    const IntSet allvidxs = _model->getVertexIds();    // Copy out since changing
    BOOST_FOREACH ( int vidx, allvidxs)
    {
        const IntSet& sfids = _model->getFaceIds(vidx);
        if ( sfids.size() != 3)
            continue;

        // Get the three face IDs.
        IntSet::const_iterator fit = sfids.begin();
        fids[0] = *fit;
        fids[1] = *(++fit);
        fids[2] = *(++fit);

        const IntSet& cvs = _model->getConnectedVertices(vidx); // Get the vertices that connect to vidx.
        // If there are four, vidx is a boundary vertex so it's ignored.
        if ( cvs.size() == 4)
            continue;

        assert( cvs.size() == 3);
        IntSet::const_iterator it = cvs.begin();
        vidxs[0] = *it;
        vidxs[1] = *(++it);
        vidxs[2] = *(++it);

        // Is there an existing face as the base of these three polygons?
        // If so, don't need to add one and can rely upon existing material setting.
        bfid = _model->getFaceId( vidxs[0], vidxs[1], vidxs[2]);
        if ( bfid < 0)
        {
            // Set material info based on the three polygons (they must shared the same material)
            const int m0 = _model->getFaceMaterialId(fids[0]);
            const int m1 = _model->getFaceMaterialId(fids[1]);
            const int m2 = _model->getFaceMaterialId(fids[2]);

            bfid = _model->setFace( vidxs);

            // Do we need to assign material offsets for this new face?
            if ( m0 >= 0 && m0 == m1 && m1 == m2)
            {
                // Find which of the texture offsets from polygon fids[0] to use (and in what order)
                const int* f0vorder = _model->getFaceVertices( fids[0]);
                const int* uvis0 = _model->getFaceUVs( fids[0]);

                IntSet cvs2 = cvs;  // Copy out
                int fnvorder[3];
                cv::Vec2f tnoffset[3];
                int j = 0;   // Will be position in fnvorder/tnoffset that needs an offset stored via reference of one of the other two polygons
                for ( int i = 0; i < 3; ++i)
                {
                    if ( f0vorder[i] == vidx)
                        j = i;
                    else
                    {
                        tnoffset[i] = _model->uv( m0, uvis0[i]);
                        fnvorder[i] = f0vorder[i];
                        cvs2.erase( f0vorder[i]);
                    }   // end if
                }   // end for

                const int vj = *cvs2.begin();
                fnvorder[j] = vj;
                // One of the other two polygons owns the texture offset for vertex fnvorder[j]
                const int* f1vorder = _model->getFaceVertices(fids[1]);
                const int* uvis1 = _model->getFaceUVs(fids[1]);
                for ( int i = 0; i < 3; ++i)
                {
                    if ( f1vorder[i] == vj)
                    {
                        tnoffset[j] = _model->uv( m1, uvis1[i]);
                        break;
                    }   // end if
                }   // end for

                _model->setOrderedFaceUVs( m0, bfid, fnvorder, tnoffset);
            }   // end if
        }   // end if

        // Remove the three faces
        _model->removeFace( fids[0]);
        _model->removeFace( fids[1]);
        _model->removeFace( fids[2]);
        _model->removeVertex( vidx); // Remove the shared vertex
        removedVertexCount++;
    }   // end foreach 

    return removedVertexCount;
}   // end removeTetrahedrons
