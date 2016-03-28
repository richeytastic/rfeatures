#include "ObjModelCleaner.h"
using RFeatures::ObjModelCleaner;
using RFeatures::ObjModel;
using RFeatures::ObjPoly;
using RFeatures::Edge;
#include "ObjModelBoundaryFinder.h"
#include "FeatureUtils.h"


// Linearly search for the face that is shared between uvidx0 and uvidx1
// that has the lowest connectivity metric (or equal lowest with another face)
// (i.e., is connected to the smallest number of other faces).
int getMinConnectivitySharedFaceId( const ObjModel::Ptr model, int uvidx0, int uvidx1, int* fcsum=NULL)
{
    // Get the IDs of the faces shared between these vertices.
    const IntSet& sharedFaceIds = model->getSharedFaces(uvidx0, uvidx1);
    assert( !sharedFaceIds.empty());

    // Of these faces, linearly search for the one with minimum connections
    // to other vertices. We prioritise removal of these faces because their
    // removal will have the least impact.
    IntSet::const_iterator it = sharedFaceIds.begin();
    int bfid = *it;
    int csum = model->getFaceConnectivityMetric( bfid);
    it++;
    while ( it != sharedFaceIds.end())
    {
        const int nsum = model->getFaceConnectivityMetric(*it);
        if ( nsum <= csum)
        {
            csum = nsum;
            bfid = *it;
        }   // end if
        ++it;
    }   // end while

    if ( fcsum != NULL)
        *fcsum = csum;

    return bfid;
}   // end getMinConnectivitySharedFaceId


ObjModelCleaner::ObjModelCleaner( ObjModel::Ptr model) : _model(model)
{}   // end ctor


size_t ObjModelCleaner::getNumLonelyVertices() const
{
    size_t count = 0;
    const IntSet& uvidxs = _model->getUniqueVertexIds();
    BOOST_FOREACH ( const int& uvidx, uvidxs)
    {
        if ( _model->isLonely(uvidx))
            count++;
    }   // end foreach
    return count;
}   // end getNumLonelyVertices


void ObjModelCleaner::removeTipsAndJunctions()
{
    IntSet uvidxs = _model->getUniqueVertexIds(); // Copied out because will be changed!

    int uvidx;
    while ( !uvidxs.empty())
    {
        uvidx = *uvidxs.begin();
        uvidxs.erase(uvidx);    // Ensure vertex isn't checked again (unless by being exposed)

        if ( _model->isJunction(uvidx))
        {
            const IntSet& cuvtxs = _model->getConnectedUniqueVertices( uvidx);

            // Ensure vertices connected to this one are checked afterwards
            // since they may have been exposed as lonely or tip vertices.
            BOOST_FOREACH ( const int& cuv, cuvtxs)
                uvidxs.insert(cuv);

            // Continue to find and remove the lowest connectivity faces
            // joined to this vertex until it is no longer a junction vertex.
            do 
            {
                int mfid = -1;
                int msum = INT_MAX;
                BOOST_FOREACH ( const int& cuv, cuvtxs)
                {
                    int csum;
                    const int fid = getMinConnectivitySharedFaceId( _model, uvidx, cuv, &csum);
                    if ( csum <= msum)
                    {
                        msum = csum;
                        mfid = fid;
                    }   // end if
                }   // end foreach
                assert( mfid != -1);
                _model->unsetFace(mfid);    // Will cause membership of cuvtxs to update too
            } while ( _model->isJunction(uvidx));
            // Sometimes this creates tips (but not lonely vertices) - still need to figure out why!
        }   // end if

        if ( _model->isLonely(uvidx))
            _model->removeUniqueVertex(uvidx);
        else if ( _model->isTip(uvidx))
        {
            // Connected vertex indices need to be checked after removing the face and the vertex.
            const IntSet& cuvtxs = _model->getConnectedUniqueVertices( uvidx);
            assert( cuvtxs.size() == 2);
            assert( _model->getFaceIdsFromUniqueVertex( uvidx).size() == 1);
            BOOST_FOREACH ( const int& cuvidx, cuvtxs)
                uvidxs.insert( cuvidx);
            // Remove the vertex's face single face
            _model->unsetFace( *_model->getFaceIdsFromUniqueVertex( uvidx).begin());
            _model->removeUniqueVertex( uvidx);
        }   // end else if
    }   // end while
}   // end removeTipsAndJunctions


// A vertex (and all of its connected faces) is removable if all of its
// connected vertices are non-flat.
bool isRemovable( const ObjModel::Ptr model, int uvidx)
{
    const IntSet& cverts = model->getConnectedUniqueVertices(uvidx);
    BOOST_FOREACH ( const int& cuvidx, cverts)
    {
        if ( model->isFlat(cuvidx))
            return false;
    }   // end foreach
    return true;
}   // end isRemovable


void removeVertexAndFaces( ObjModel::Ptr model, int uvidx)
{
    const IntSet fids = model->getFaceIdsFromUniqueVertex( uvidx); // Copy out
    BOOST_FOREACH ( const int& fid, fids)
        model->unsetFace( fid);
    model->removeUniqueVertex( uvidx);
}   // end removeVertexAndFaces


// public
void ObjModelCleaner::removeNonFlat()
{
    IntSet uvidxs = _model->getUniqueVertexIds(); // Copied out because will be changed!
    assert( !uvidxs.empty());
    if ( uvidxs.empty())
        return;

    int uvidx, remFaceId;
    do
    {
        uvidx = *uvidxs.begin();
        uvidxs.erase(uvidx);    // Ensure this vertex isn't checked again

        if ( isRemovable( _model, uvidx))
            removeVertexAndFaces( _model, uvidx);
        else if ( !_model->isFlat(uvidx))
        {
            // Find the other vertex (vertices) connected to this vertex that are also non-flat
            // and remove the shared faces having minimum connectivity.
            const IntSet& cuvidxs = _model->getConnectedUniqueVertices(uvidx);
            BOOST_FOREACH ( const int& cuvidx, cuvidxs)
            {
                while ( _model->getNumSharedFaces( uvidx, cuvidx) > 2)
                {
                    remFaceId = getMinConnectivitySharedFaceId( _model, uvidx, cuvidx);
                    _model->unsetFace( remFaceId);
                }   // end while
            }   // end foreach

            // Since all of the extra faces that uvidx shares with other of its connected
            // vertices have been removed at this point, uvidx should no longer be non-flat.
            assert( _model->isFlat(uvidx));
        }   // end if
    } while ( !uvidxs.empty());
}   // end removeNonFlat


// public
// Vertices in the model with < minVtxFaceConns vertex connections are removed
int ObjModelCleaner::pruneVertices( int minVtxFaceConns)
{
    int remCount = 0;
    const IntSet uvidxs = _model->getUniqueVertexIds(); // Copied out because will be changed!
    BOOST_FOREACH ( const int& ui, uvidxs)
    {
        if ( _model->getUniqueVertexFaceCount(ui) < minVtxFaceConns)
        {
            removeVertexAndFaces( _model, ui);
            remCount++;
        }   // end if
    }   // end foreach
    return remCount;
}   // end pruneVertices



bool hasTooLongEdges( const ObjModel::Ptr& model, int fid, double maxLen)
{
    const ObjPoly& face = model->getUniqueVertexFace(fid);
    const cv::Vec3f& v0 = model->getUniqueVertex( face.vindices[0]);
    const cv::Vec3f& v1 = model->getUniqueVertex( face.vindices[1]);
    const cv::Vec3f& v2 = model->getUniqueVertex( face.vindices[2]);
    return cv::norm(v0 - v1) > maxLen || cv::norm(v1 - v2) > maxLen || cv::norm(v2 - v0) > maxLen;
}   // end hasTooLongEdges


// Remove from the boundary those faces having a edge greater in length than
// ulen + 0.6stddev. After this, those vertices not having any connected vertices are removed.
// public
void ObjModelCleaner::removeBoundarySpikes()
{
    std::vector<double> elens;
    const double ulen = RFeatures::calcMeanEdgeDistance( _model, elens);
    const double stddev = RFeatures::calcStdDevBiased( elens, ulen);
    const double maxLen = ulen + 0.6*stddev;

    const IntSet& uvidxs = _model->getUniqueVertexIds();
    std::vector<int> deluvidxs;
    BOOST_FOREACH ( const int& uvidx, uvidxs)
    {
        if ( !_model->isBoundary(uvidx)) // Ignore non boundary vertices
            continue;
       
        // Get all of the faces that use this boundary vertex 
        const IntSet& cfaces = _model->getFaceIdsFromUniqueVertex(uvidx);
        std::vector<int> delfaces;
        // Check the edges of each of these faces - set for removal any with too long edges.
        BOOST_FOREACH ( const int& fid, cfaces)
        {
            if ( hasTooLongEdges( _model, fid, maxLen))
                delfaces.push_back(fid);
        }   // end foreach

        // Remove the faces set for deletion
        BOOST_FOREACH ( const int& fid, delfaces)
            _model->unsetFace(fid);

        // Set the vertex to be deleted if it has no more faces attached
        if ( cfaces.empty())
            deluvidxs.push_back(uvidx);
    }   // end foreach

    // Remove the unique vertices with no more attached faces.
    BOOST_FOREACH ( const int& uvidx, deluvidxs)
        _model->removeUniqueVertex(uvidx);
}   // end removeBoundarySpikes


// public
void ObjModelCleaner::removeHighGradientVertices( double maxGrad)
{
    const IntSet& uvidxs = _model->getUniqueVertexIds();
    BOOST_FOREACH ( const int& uvidx, uvidxs)
    {
        if ( fabs( _model->getUniqueVertexCurvature(uvidx)) <= maxGrad)
            continue;

        // Get the old connected vertices to interpolate a new 3D vertex.
        const IntSet& cuvidxs = _model->getConnectedUniqueVertices(uvidx);
        cv::Vec3f nvtx(0,0,0);
        BOOST_FOREACH ( const int& cuv, cuvidxs)
            nvtx += _model->getUniqueVertex(cuv);
        nvtx *= (1.0/cuvidxs.size());

        // Get all the old texture indices and map to the new indices.
        const IntSet& txIndices = _model->lookupTextureIndices( uvidx);
        boost::unordered_map<int, int> oldToNewTxIndices;
        int ntxidx = -1;
        BOOST_FOREACH ( const int& txidx, txIndices)
        {
            const cv::Vec2f& tx = _model->getTextureOffset(txidx);
            ntxidx = _model->addVertex( nvtx, tx);
            oldToNewTxIndices[txidx] = ntxidx;
        }   // end foreach
        assert(ntxidx != -1);
        const int nuvidx = _model->getUniqueVertexIdFromNonUnique(ntxidx);   // The new unique vertex ID

        // Get all the old faces connected to this unique vertex.
        // Set replacement faces using the same texture coordinates
        // and other 3D vertices apart from for the identified vertex.
        const IntSet fids = _model->getFaceIdsFromUniqueVertex( uvidx); // Copy out for changing
        BOOST_FOREACH ( const int& fid, fids)
        {
            const ObjPoly& poly = _model->getFace(fid); // Note, this is the texture face!

            // Remap the texture vertex index for the appropriate vertex
            int v0 = poly.vindices[0];
            if ( oldToNewTxIndices.count(v0))
                v0 = oldToNewTxIndices[v0];
            int v1 = poly.vindices[1];
            if ( oldToNewTxIndices.count(v1))
                v1 = oldToNewTxIndices[v1];
            int v2 = poly.vindices[2];
            if ( oldToNewTxIndices.count(v2))
                v2 = oldToNewTxIndices[v2];

            _model->setFace( v0, v1, v2); // Set the the replacement face
            _model->unsetFace(fid); // Delete the old face
        }   // end foreach

        _model->removeUniqueVertex(uvidx);  // Remove the old unique vertex

        // Since the gradient has changed on the vertex, we need to recheck all
        // the connected vertices since those gradients will have been affected.
        uvidxs.insert(cuvidxs.begin(), cuvidxs.end());
        //uvidxs.insert(nuvidx);  // Also add the newly adjusted vertex
    }   // end foreach
}   // end removeHighGradientVertices


// public
void ObjModelCleaner::removeUniqueVertices( const IntSet& uvidxs)
{
    BOOST_FOREACH ( const int& uvidx, uvidxs)
        removeVertexAndFaces( _model, uvidx);
}   // end removeVertices


// public
int ObjModelCleaner::fillHoles()    // TODO
{
    /*
    using RFeatures::ObjModelBoundaryFinder;
    ObjModelBoundaryFinder* ombf = new ObjModelBoundaryFinder( _model);
    const int nbs = ombf->findOrderedBoundaryUniqueVertexIndices(); // Returns number of boundaries
    delete ombf;
    return nbs;
    */
    return 1;
}   // end fillHoles


