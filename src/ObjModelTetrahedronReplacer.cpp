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
                const ObjModel::Material& mat = _model->getMaterial(m0);
                // Find which of the texture offsets from polygon fids[0] to use (and in what order)
                const cv::Vec3i& f0vorder = mat.faceVertexOrder.at(fids[0]);
                const cv::Vec6f& t0offset = mat.txOffsets.at(fids[0]);

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
                        tnoffset[i] = cv::Vec2f( t0offset[2*i], t0offset[2*i+1]);
                        fnvorder[i] = f0vorder[i];
                        cvs2.erase( f0vorder[i]);
                    }   // end if
                }   // end for

                const int vj = *cvs2.begin();
                fnvorder[j] = vj;
                // One of the other two polygons owns the texture offset for vertex fnvorder[j]
                const cv::Vec3i& f1vorder = mat.faceVertexOrder.at(fids[1]);
                const cv::Vec6f& t1offset = mat.txOffsets.at(fids[1]);
                for ( int i = 0; i < 3; ++i)
                {
                    if ( f1vorder[i] == vj)
                    {
                        tnoffset[j] = cv::Vec2f( t1offset[2*i], t1offset[2*i+1]);
                        break;
                    }   // end if
                }   // end for

                _model->setFaceTextureOffsets( m0, bfid, fnvorder[0], tnoffset[0],
                                                         fnvorder[1], tnoffset[1],
                                                         fnvorder[2], tnoffset[2]);
            }   // end if
        }   // end if

        // Remove the three faces
        _model->unsetFace( fids[0]);
        _model->unsetFace( fids[1]);
        _model->unsetFace( fids[2]);
        _model->removeVertex( vidx); // Remove the shared vertex
        removedVertexCount++;
    }   // end foreach 

    return removedVertexCount;
}   // end removeTetrahedrons
