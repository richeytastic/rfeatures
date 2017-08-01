#include "ObjModelCleaner.h"
using RFeatures::ObjModelCleaner;
using RFeatures::ObjModel;
using RFeatures::ObjPoly;
#include "ObjModelTopologyFinder.h"
#include "FeatureUtils.h"
#include <boost/foreach.hpp>
#include <cstring>


// Linearly search for the face that is shared between v0 and v1
// that has the lowest connectivity metric (or equal lowest with another face)
// (i.e., is connected to the smallest number of other faces).
int getMinConnectivitySharedFaceId( const ObjModel::Ptr model, int v0, int v1, int* fcsum=NULL)
{
    // Get the IDs of the faces shared between these vertices.
    const IntSet& sharedFaceIds = model->getSharedFaces(v0, v1);
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


void ObjModelCleaner::updateVertexTopology( int vidx)
{
    _flat->erase(vidx);
    _lonely->erase(vidx);  // Shares no edges (disconnected)
    _nonflat->erase(vidx);
    _edge->erase(vidx);
    _comp->erase(vidx);

    if ( !_model->getVertexIds().count(vidx))
        return;

    RFeatures::ObjModelTopologyFinder omtf( _model);
    RFeatures::ObjModelTopologyFinder::BasicTopology btopology = omtf.getBasicTopology( vidx);
    if ( btopology & ObjModelTopologyFinder::VTX_UNCONNECTED)
        _lonely->insert(vidx);
    else
    {
        RFeatures::ObjModelTopologyFinder::ComplexTopology ctopology = omtf.getComplexTopology( vidx);
        if ( ctopology & ObjModelTopologyFinder::VTX_FLAT)
            _flat->insert(vidx);
        else
            _nonflat->insert(vidx);

        if ( ctopology & ObjModelTopologyFinder::VTX_EDGE)
            _edge->insert(vidx);
        if ( ctopology & ObjModelTopologyFinder::VTX_COMPLETE)
            _comp->insert(vidx);
    }   // end else
}   // end updateVertexTopology


// Obtains unique vertex topology values over the whole model.
void ObjModelCleaner::regatherTopology()
{
    _flat->clear();
    _lonely->clear();
    _nonflat->clear();
    _edge->clear();
    _comp->clear();
    const IntSet& vidxs = _model->getVertexIds();
    BOOST_FOREACH ( int vidx, vidxs)
        updateVertexTopology( vidx);
}   // end regatherTopology


ObjModelCleaner::ObjModelCleaner( ObjModel::Ptr model)
    : _progressDelegate(NULL), _model(model),
    _lonely( new IntSet), _flat( new IntSet), _nonflat( new IntSet),
    _edge( new IntSet), _comp( new IntSet)
{
    regatherTopology();
}   // end ctor


ObjModelCleaner::~ObjModelCleaner()
{
    delete _lonely;
    delete _flat;
    delete _nonflat;
    delete _edge;
    delete _comp;
}   // end dtor


void updateProgressDelegate( rlib::ProgressDelegate* pd, float pc)
{
    if ( pd)
        pd->updateProgress(pc);
}   // end updateProgressDelegate


// public
void ObjModelCleaner::setProgressDelegate( rlib::ProgressDelegate* pd)
{
    _progressDelegate = pd;
}   // end setProgressDelegate


// private
bool ObjModelCleaner::removeVertex( int vidx)
{
    const IntSet& vidxs = _model->getVertexIds();
    if ( vidxs.count(vidx) == 0)
        return false;
    assert( _model->getFaceIds(vidx).size() == 0);
    _flat->erase(vidx);
    _lonely->erase(vidx);
    _nonflat->erase(vidx);
    _edge->erase(vidx);
    _comp->erase(vidx);
    const bool removedOkay = _model->removeVertex( vidx);
    assert(removedOkay);
    return removedOkay;
}   // end removeVertex


// private
void ObjModelCleaner::removeFace( int fid)
{
    if ( !_model->getFaceIds().count(fid))
        return;

    const ObjPoly& face = _model->getFace( fid);
    int vtx[3];
    memcpy( vtx, &face.vindices[0], 3*sizeof(int));
    _model->unsetFace( fid);

    if ( _model->getFaceIds(vtx[0]).size() == 0) removeVertex(vtx[0]);
    else updateVertexTopology(vtx[0]);

    if ( _model->getFaceIds(vtx[1]).size() == 0) removeVertex(vtx[1]);
    else updateVertexTopology(vtx[1]);

    if ( _model->getFaceIds(vtx[2]).size() == 0) removeVertex(vtx[2]);
    else updateVertexTopology(vtx[2]);
}   // end removeFace


// private
void ObjModelCleaner::removeVertexAndFaces( int vidx)
{
    if ( !_model->getVertexIds().count(vidx))
        return;

    int v0, v1;
    IntSet vtxUpdate;
    const IntSet fids = _model->getFaceIds( vidx); // Copy out
    BOOST_FOREACH ( int fid, fids)
    {
        const ObjPoly& face = _model->getFace( fid);
        face.getOpposite( vidx, v0, v1);
        vtxUpdate.insert( v0);
        vtxUpdate.insert( v1);
        _model->unsetFace( fid);
    }   // end foreach

    removeVertex( vidx);
    vtxUpdate.erase( vidx);
    BOOST_FOREACH ( int v2, vtxUpdate)
        updateVertexTopology(v2);
}   // end removeVertexAndFaces


// private
// A vertex (and all of its connected faces) is removable if all of its
// connected vertices have non-flat topology.
bool ObjModelCleaner::is3DExtrusion( int vidx) const
{
    const IntSet& cverts = _model->getConnectedVertices(vidx);
    BOOST_FOREACH ( int cvidx, cverts)
    {
        if ( _flat->count(cvidx))
            return false;
    }   // end foreach
    return true;
}   // end is3DExtrusion


// private
int ObjModelCleaner::removeNonFlatMakingEdges( int vidx)
{
    int removed = 0;
    // Find the connected vertices that vidx makes edges with, and remove these
    // until vidx is no longer a member of _nonflat.
    const IntSet cvidxs = _model->getConnectedVertices(vidx);
    BOOST_FOREACH ( int cv, cvidxs)
    {
        if ( _model->getNumSharedFaces( vidx, cv) <= 0)   // Dealt with already
            continue;

        const IntSet& fids = _model->getSharedFaces( vidx, cv);
        if ( fids.size() == 1)
        {
            const int fid = *fids.begin();
            removeFace( fid); // Updates topology info for vidx as well
            removed++;
            if ( !_nonflat->count(vidx))
                break;
        }   // end if
    }   // end foreach
    return removed;
}   // end removeNonFlatMakingEdges


// private
int ObjModelCleaner::removeFlatExtrusions()
{
    int remCount = 0;
    IntSet vtxs = *_flat;   // Copy out because changing
    BOOST_FOREACH ( int vidx, vtxs)
    {
        // If all of vidx's connected vertices are 3D, then it can be safely removed.
        if ( is3DExtrusion( vidx))
        {
            removeVertexAndFaces( vidx);
            remCount++;
        }   // end if
    }   // end foreach
    return remCount;
}   // end removeFlatExtrusions


// private
void ObjModelCleaner::removeTetrahedronBases()
{
    int v0, v1, v2, fid;
    BOOST_FOREACH ( int vidx, *_flat)
    {
        const IntSet& cvidxs = _model->getConnectedVertices(vidx);
        if ( cvidxs.size() == 3 && is3DExtrusion( vidx))
        {
            IntSet::const_iterator it = cvidxs.begin();
            v0 = *it;
            v1 = *(++it);
            v2 = *(++it);
            fid = _model->getFaceId( v0, v1, v2);
            if ( fid >= 0)
                removeFace(fid);
        }   // end if
    }   // end foreach
}   // end removeTetrahedronBases


// private
// Vertices not in the _edge, _comp, and flat sets, or
// vertices in the _edge set but not in the _flat or _comp sets.
int ObjModelCleaner::removeJunctionConnections( int vidx)
{
    // vidx separates distinct sets of connected vertices.
    IntSet cvidxs = _model->getConnectedVertices( vidx);  // Copy out
    // We remove all of these except the largest set since this is assumed to be the
    // most important in describing the local region of which vertex vidx is a part.
    assert(!cvidxs.empty());

    // Separate cvidxs into separate connected sets
    std::vector<IntSet> csets(1);  // Separate sets (initially there's only one, but there WILL be at least two)
    int largestSetIdx = 0;  // Index into csets of the largest set

    int s0 = *cvidxs.begin();  // Initial set seed
    std::vector<int> setExpandFront;    // The vertices to expand next
    setExpandFront.push_back(s0);
    while ( !setExpandFront.empty())
    {
        IntSet& cset = csets.back(); // Set we're adding to with the current front (last member of csets)

        s0 = setExpandFront.back();
        setExpandFront.pop_back();
        cvidxs.erase(s0);

        // Collect all of the connected vertices to expand next (if not already in cset)
        const IntSet& fids = _model->getSharedFaces( vidx, s0);    // Typically just two but could be more...
        BOOST_FOREACH ( int fid, fids)
        {
            const ObjPoly& poly = _model->getFace(fid);
            const int otherv = poly.getOpposite( vidx, s0);
            if ( !cset.count(otherv))
            {
                cset.insert(otherv);
                setExpandFront.push_back(otherv);
                // Check if current connected set is now the largest
                if ( cset.size() >= csets[largestSetIdx].size())
                    largestSetIdx = (int)csets.size()-1;
            }   // end if
        }   // end foreach

        // If the expansion front has been exhausted, but there are still connected vertices in cvidxs
        // we need to add in another connected set.
        if ( setExpandFront.empty() && !cvidxs.empty())
        {
            csets.resize( csets.size()+1);
            setExpandFront.push_back( *cvidxs.begin());
        }   // end if
    }   // end while

    int removedCount = 0;
    const int nsets = (int)csets.size();
    for ( int i = 0; i < nsets; ++i)
    {
        if ( i == largestSetIdx)    // Keep this set!
            continue;

        // Remove all faces shared by every connected vertex in csets[i]
        const IntSet& rejectSet = csets[i];
        BOOST_FOREACH ( int cv, rejectSet)
        {
            if ( _model->getNumSharedFaces( vidx, cv) <= 0)   // Dealt with already
                continue;
            const IntSet fids = _model->getSharedFaces( vidx, cv);    // Copy out because changing
            BOOST_FOREACH ( int fid, fids)
                removeFace( fid);
            removedCount++;
        }   // end foreach
    }   // end for

    return removedCount;
}   // end removeJunctionConnections


// private
int ObjModelCleaner::removeSurfaceJoin( int vidx)
{
    IntSet rmfids;
    const IntSet& cvidxs = _model->getConnectedVertices( vidx);
    BOOST_FOREACH ( int cv, cvidxs)
    {   
        if ( _model->getNumSharedFaces( vidx, cv) > 2)
        {
            const IntSet& fids = _model->getSharedFaces( vidx, cv);
            rmfids.insert( fids.begin(), fids.end());
        }   // end if
    }   // end foreach

    // Will create a hole
    BOOST_FOREACH ( int fid, rmfids)
        removeFace( fid);

    return 0;
}   // end removeSurfaceJoin


// public
int ObjModelCleaner::remove3D()
{
    int nonFlatEdgeRemovals = 0;
    int junctionARemovals = 0;
    int junctionBRemovals = 0;
    int surfaceJoinRemovals = 0;
    int removedExtrusions = 0;

    const IntSet& vidxs = _model->getVertexIds();

    bool loopExtra = true;  // Use one extra loop
    int numNonFlat = (int)_nonflat->size();
    int oldNumNonFlat = numNonFlat > 0 ? numNonFlat+1 : numNonFlat;
    while ( numNonFlat < oldNumNonFlat || loopExtra)
    {
        loopExtra = numNonFlat != oldNumNonFlat;   // One extra loop whenever no change

        oldNumNonFlat = numNonFlat;
        //removedExtrusions += removeFlatExtrusions();
        removeTetrahedronBases();

        IntSet vtxs = *_nonflat;   // Copy out because changing
        BOOST_FOREACH ( int vidx, vtxs)
        {
            // Recheck since removal of polys might've affected this vertex.
            if ( _flat->count(vidx) || !vidxs.count(vidx))
                continue;

            if ( _edge->count(vidx))
            {   // At least one vertex connected to vidx shares only a single poly with vidx
                if ( _comp->count(vidx))
                    nonFlatEdgeRemovals += removeNonFlatMakingEdges( vidx);
                else
                    junctionARemovals += removeJunctionConnections( vidx);
            }   // end if
            else
            {   // All vertices connected to vidx share 2 or more polys with vidx.
                // This can take two different forms.
                if ( _comp->count(vidx))
                    surfaceJoinRemovals += removeSurfaceJoin( vidx);
                else
                    junctionBRemovals += removeJunctionConnections( vidx);
            }   // end else
        }   // end foreach

        numNonFlat = (int)_nonflat->size();
    }   // end while

    return nonFlatEdgeRemovals + junctionARemovals + junctionBRemovals + surfaceJoinRemovals + removedExtrusions;
}   // end remove3D


// public
int ObjModelCleaner::remove1D()
{
    const IntSet& vidxs = _model->getVertexIds();

    int remTotal = 0;
    int nremoved = 0;

    do
    {
        std::vector<int> remSet; // The remove set is edge and flat vertices that aren't complete
        BOOST_FOREACH ( int vidx, *_edge)
        {
            if ( _flat->count(vidx) && !_comp->count(vidx))
                remSet.push_back(vidx);
        }   // end foreach

        nremoved = 0;
        BOOST_FOREACH ( int vidx, remSet)
        {
            if ( !vidxs.count(vidx))
                continue;
            nremoved += removeJunctionConnections( vidx);
        }   // end foreach

        // Removing some faces may have meant that some vertices became lonely,
        // so removal of the lonely vertices happens at the end of this function.
        IntSet vtxs = *_lonely;   // Copy out because changing
        BOOST_FOREACH ( int vidx, vtxs)
        {
            if ( removeVertex(vidx))
                nremoved++;
        }   // end for

        remTotal += nremoved;
    } while ( nremoved > 0);

    return remTotal;
}   // end remove1D


// public
// Vertices in the model with < minVtxFaceConns vertex connections are removed
int ObjModelCleaner::pruneVertices( int minVtxFaceConns)
{
    int remCount = 0;
    const IntSet vidxs = _model->getVertexIds(); // Copied out because will be changed!
    BOOST_FOREACH ( int vi, vidxs)
    {
        if ( _model->getVertexFaceCount(vi) < minVtxFaceConns)
        {
            removeVertexAndFaces( vi);
            remCount++;
        }   // end if
    }   // end foreach
    return remCount;
}   // end pruneVertices


// public
void ObjModelCleaner::removeVertices( const IntSet& vidxs)
{
    BOOST_FOREACH ( int vidx, vidxs)
        removeVertexAndFaces( vidx);
}   // end removeVertices

