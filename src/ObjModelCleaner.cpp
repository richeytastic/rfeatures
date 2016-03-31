#include "ObjModelCleaner.h"
using RFeatures::ObjModelCleaner;
using RFeatures::ObjModel;
using RFeatures::ObjPoly;
using RFeatures::Edge;
#include "ObjModelBoundaryFinder.h"
#include "FeatureUtils.h"
#include <queue>    // std::priority_queue


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


// For comparing gradients between vertices when using heaps.
class UniqueVertexGradientComparator
{
    const ObjModel::Ptr _model;
    bool _reverse;

public:
    UniqueVertexGradientComparator( const ObjModel::Ptr om, bool reverse=false)
        : _model(om), _reverse(reverse) {}

    bool operator()( const int& ui, const int& uj) const
    {
        const double gi = _model->getUniqueVertexCurvature(ui);
        const double gj = _model->getUniqueVertexCurvature(uj);
        if ( _reverse)
            return gi > gj;
        return gi < gj;
    }   // end operator()
};  // end class



cv::Vec3f interpolateVertexFromConnected( const ObjModel::Ptr om, int uvidx)
{
    const IntSet& cuvidxs = om->getConnectedUniqueVertices(uvidx);
    cv::Vec3f nvtx(0,0,0);
    assert(!cuvidxs.empty());
    BOOST_FOREACH ( const int& cuv, cuvidxs)
        nvtx += om->getUniqueVertex(cuv);
    nvtx *= (1.0/cuvidxs.size());
    return nvtx;
}   // end interpolateVertexFromConnected


// Takes all of the old texture vertex indices that map to the given unique vertex ID
// and creates new texture vertices using the provided new 3D vertex, taking the
// returned vertex IDs and mapping them to the old vertex IDs (which remain in the
// provided model for the moment).
int mapOldToNewTextureVertices( ObjModel::Ptr om, int uvidx,
                                const cv::Vec3f& nvtx, boost::unordered_map<int, int>& oldToNewTxIndices)
{
    const IntSet& txIndices = om->lookupTextureIndices( uvidx);
    int ntxidx = -1;
    assert(!txIndices.empty());
    BOOST_FOREACH ( const int& txidx, txIndices)
    {
        const cv::Vec2f& tx = om->getTextureOffset(txidx);
        ntxidx = om->addVertex( nvtx, tx);
        oldToNewTxIndices[txidx] = ntxidx;
    }   // end foreach
    assert(ntxidx != -1);
    return om->getUniqueVertexIdFromNonUnique(ntxidx);
}   // end mapOldToNewTextureVertices


// Get all the old faces connected to this unique vertex.
// Set replacement faces using the same texture coordinates
// and other 3D vertices apart from for the identified vertex.
void adjustFaces( ObjModel::Ptr om, int uvidx, const boost::unordered_map<int,int>& oldToNewTxIndices)
{
    const IntSet fids = om->getFaceIdsFromUniqueVertex( uvidx); // Copy out for changing
    BOOST_FOREACH ( const int& fid, fids)
    {
        const ObjPoly& poly = om->getFace(fid); // Note, this is the texture face!

        // Remap the texture vertex index for the appropriate vertex
        int v0 = poly.vindices[0];
        if ( oldToNewTxIndices.count(v0))
            v0 = oldToNewTxIndices.at(v0);
        int v1 = poly.vindices[1];
        if ( oldToNewTxIndices.count(v1))
            v1 = oldToNewTxIndices.at(v1);
        int v2 = poly.vindices[2];
        if ( oldToNewTxIndices.count(v2))
            v2 = oldToNewTxIndices.at(v2);

        om->setFace( v0, v1, v2); // Set the the replacement face
        om->unsetFace(fid); // Delete the old face
    }   // end foreach
}   // end adjustFaces


// public
void ObjModelCleaner::adjustHighGradientVertices( double maxGrad)
{
    std::vector<int> remUvidxs;   // Will hold unique vertices that are no longer needed.
    const IntSet& uvidxs = _model->getUniqueVertexIds();
    BOOST_FOREACH ( const int& uvidx, uvidxs)
    {
        if ( fabs( _model->getUniqueVertexCurvature(uvidx)) <= maxGrad)
            continue;

        // Get the old connected vertices to interpolate a new 3D vertex.
        const cv::Vec3f nvtx = interpolateVertexFromConnected( _model, uvidx);

        // Map all the old texture indices to newly added indices.
        boost::unordered_map<int, int> oldToNewTxIndices;
        const int nuvidx = mapOldToNewTextureVertices( _model, uvidx, nvtx, oldToNewTxIndices);  // The new unique vertex ID

        // Take all the faces connected to uvidx and set them to use the new texture indices.
        // This removes the old faces but does NOT remove the unique vertex uvidx from the model
        // (which is no longer required since no faces reference it).
        adjustFaces( _model, uvidx, oldToNewTxIndices);
        remUvidxs.push_back(uvidx);    // For removal (can't do here because iterating over reference container)
    }   // end foreach

    // Remove the unique vertices that are no longer used by any faces.
    BOOST_FOREACH ( const int& uvidx, remUvidxs)
        _model->removeUniqueVertex(uvidx);
}   // end adjustHighGradientVertices


// public
void ObjModelCleaner::removeUniqueVertices( const IntSet& uvidxs)
{
    BOOST_FOREACH ( const int& uvidx, uvidxs)
        removeVertexAndFaces( _model, uvidx);
}   // end removeVertices


// public
int ObjModelCleaner::fillHoles()    // TODO
{
    using RFeatures::ObjModelBoundaryFinder;
    ObjModelBoundaryFinder* ombf = new ObjModelBoundaryFinder( _model);
    const int nbs = ombf->findOrderedBoundaryUniqueVertexIndices(); // Returns number of boundaries
    std::cerr << "Found " << nbs << " boundaries on model" << std::endl;
    for ( int i = 0; i < nbs; ++i)
    {
        const std::list<int>& boundary = ombf->getBoundary(i);
        std::cerr << " + Boundary " << i << " has vertex length " << boundary.size() << std::endl;
    }   // end for
    delete ombf;
    return nbs;
}   // end fillHoles


