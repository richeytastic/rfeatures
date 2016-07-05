#include "ObjModelCleaner.h"
using RFeatures::ObjModelCleaner;
using RFeatures::ObjModel;
using RFeatures::ObjPoly;
using RFeatures::Edge;
#include "ObjModelBoundaryFinder.h"
#include "ObjModelTopologyFinder.h"
#include "FeatureUtils.h"
#include <queue>    // std::priority_queue
#include <boost/foreach.hpp>


// Linearly search for the face that is shared between uv0 and uv1
// that has the lowest connectivity metric (or equal lowest with another face)
// (i.e., is connected to the smallest number of other faces).
int getMinConnectivitySharedFaceId( const ObjModel::Ptr model, int uv0, int uv1, int* fcsum=NULL)
{
    // Get the IDs of the faces shared between these vertices.
    const IntSet& sharedFaceIds = model->getSharedFaces(uv0, uv1);
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


void ObjModelCleaner::updateUniqueVertexTopology( int uvidx)
{
    _flat->erase(uvidx);
    _lonely->erase(uvidx);
    _nonflat->erase(uvidx);
    _edge->erase(uvidx);
    _comp->erase(uvidx);

    if ( !_model->getUniqueVertexIds().count(uvidx))
        return;

    RFeatures::ObjModelTopologyFinder omtf( _model);
    RFeatures::ObjModelTopologyFinder::VertexTopology vtopology = omtf.discoverLocalTopology( uvidx);

    if ( vtopology & ObjModelTopologyFinder::VTX_UNCONNECTED)
        _lonely->insert(uvidx);
    else
    {
        if ( vtopology & ObjModelTopologyFinder::VTX_FLAT)
            _flat->insert(uvidx);
        else
            _nonflat->insert(uvidx);

        if ( vtopology & ObjModelTopologyFinder::VTX_EDGE)
            _edge->insert(uvidx);
        if ( vtopology & ObjModelTopologyFinder::VTX_COMPLETE)
            _comp->insert(uvidx);
    }   // end else
}   // end updateUniqueVertexTopology


// Obtains unique vertex topology values over the whole model.
void ObjModelCleaner::regatherTopology()
{
    _flat->clear();
    _lonely->clear();
    _nonflat->clear();
    _edge->clear();
    _comp->clear();
    const IntSet& uvidxs = _model->getUniqueVertexIds();
    BOOST_FOREACH ( const int& uvidx, uvidxs)
        updateUniqueVertexTopology( uvidx);
}   // end regatherTopology


ObjModelCleaner::ObjModelCleaner( ObjModel::Ptr model) : _model(model),
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


/*
void ObjModelCleaner::removeTips()
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
}   // end removeTips
*/


// private
bool ObjModelCleaner::removeVertex( int uvidx)
{
    const IntSet& uvidxs = _model->getUniqueVertexIds();
    if ( !uvidxs.count(uvidx))
        return false;
    assert( _model->getFaceIdsFromUniqueVertex(uvidx).size() == 0);
    _flat->erase(uvidx);
    _lonely->erase(uvidx);
    _nonflat->erase(uvidx);
    _edge->erase(uvidx);
    _comp->erase(uvidx);
    _model->removeUniqueVertex( uvidx);
    return true;
}   // end removeVertex


// private
void ObjModelCleaner::removeFace( int fid)
{
    if ( !_model->getFaceIds().count(fid))
        return;

    std::vector<int> uvtxUpdate(3);
    const ObjPoly& face = _model->getUniqueVertexFace( fid);
    uvtxUpdate[0] = face.vindices[0];
    uvtxUpdate[1] = face.vindices[1];
    uvtxUpdate[2] = face.vindices[2];
    _model->unsetFace( fid);
    BOOST_FOREACH ( const int& uvidx, uvtxUpdate)
    {
        if ( _model->getFaceIdsFromUniqueVertex(uvidx).size() == 0)
            removeVertex(uvidx);
        else
            updateUniqueVertexTopology(uvidx);
    }   // end foreach
}   // end removeFace


// private
void ObjModelCleaner::removeVertexAndFaces( int uvidx)
{
    if ( !_model->getUniqueVertexIds().count(uvidx))
        return;

    IntSet uvtxUpdate;
    const IntSet fids = _model->getFaceIdsFromUniqueVertex( uvidx); // Copy out
    BOOST_FOREACH ( const int& fid, fids)
    {
        const ObjPoly& face = _model->getUniqueVertexFace( fid);
        uvtxUpdate.insert(face.vindices[0]);
        uvtxUpdate.insert(face.vindices[1]);
        uvtxUpdate.insert(face.vindices[2]);
        _model->unsetFace( fid);
    }   // end foreach

    removeVertex( uvidx);
    uvtxUpdate.erase(uvidx);
    BOOST_FOREACH ( const int& uv2, uvtxUpdate)
        updateUniqueVertexTopology(uv2);
}   // end removeVertexAndFaces


// A vertex (and all of its connected faces) is removable if all of its
// connected vertices have non-flat topology.
bool ObjModelCleaner::is3DExtrusion( int uvidx) const
{
    const IntSet& cverts = _model->getConnectedUniqueVertices(uvidx);
    BOOST_FOREACH ( const int& cuvidx, cverts)
    {
        if ( _flat->count(cuvidx))
            return false;
    }   // end foreach
    return true;
}   // end is3DExtrusion


// private
int ObjModelCleaner::removeNonFlatMakingEdges( int uvidx)
{
    const RFeatures::ObjModelTopologyFinder modelTopologyFinder( _model);

    int removed = 0;
    // Find the connected vertices that uvidx makes edges with, and remove these
    // until uvidx is no longer a member of _nonflat.
    const IntSet cuvidxs = _model->getConnectedUniqueVertices(uvidx);
    BOOST_FOREACH ( const int& cuv, cuvidxs)
    {
        if ( _model->getNumSharedFaces( uvidx, cuv) <= 0)   // Dealt with already
            continue;

        const IntSet& fids = _model->getSharedFaces( uvidx, cuv);
        if ( fids.size() == 1)
        {
            const int fid = *fids.begin();
            removeFace( fid); // Updates topology info for uvidx as well
            removed++;
            if ( !_nonflat->count(uvidx))
                break;
        }   // end if
    }   // end foreach
    return removed;
}   // end removeNonFlatMakingEdges


// private
int ObjModelCleaner::removeFlatExtrusions()
{
    int remCount = 0;
    IntSet uvtxs = *_flat;   // Copy out because changing
    BOOST_FOREACH ( const int& uvidx, uvtxs)
    {
        // If all of uvidx's connected vertices are 3D, then it can be safely removed.
        if ( is3DExtrusion( uvidx))
        {
            removeVertexAndFaces( uvidx);
            remCount++;
        }   // end if
    }   // end foreach
    return remCount;
}   // end removeFlatExtrusions


// private
// Vertices not in the _edge, _comp, and flat sets, or
// vertices in the _edge set but not in the _flat or _comp sets.
int ObjModelCleaner::removeJunctionConnections( int uvidx)
{
    // uvidx separates distinct sets of connected vertices.
    IntSet cuvidxs = _model->getConnectedUniqueVertices( uvidx);  // Copy out
    // We remove all of these except the largest set since this is assumed to be the
    // most important in describing the local region of which vertex uvidx is a part.
    assert(!cuvidxs.empty());

    // Separate cuvidxs into separate connected sets
    std::vector<IntSet> csets(1);  // Separate sets (initially there's only one, but there WILL be at least two)
    int largestSetIdx = 0;  // Index into csets of the largest set

    int s0 = *cuvidxs.begin();  // Initial set seed
    std::vector<int> setExpandFront;    // The vertices to expand next
    setExpandFront.push_back(s0);
    while ( !setExpandFront.empty())
    {
        IntSet& cset = csets.back(); // Set we're adding to with the current front (last member of csets)

        s0 = setExpandFront.back();
        setExpandFront.pop_back();
        cuvidxs.erase(s0);

        // Collect all of the connected vertices to expand next (if not already in cset)
        const IntSet& fids = _model->getSharedFaces( uvidx, s0);    // Typically just two but could be more...
        BOOST_FOREACH ( const int& fid, fids)
        {
            const ObjPoly& poly = _model->getUniqueVertexFace(fid);
            const int otheruv = RFeatures::ObjModelTopologyFinder::getOtherVertex( poly, uvidx, s0);
            if ( !cset.count(otheruv))
            {
                cset.insert(otheruv);
                setExpandFront.push_back(otheruv);
                // Check if current connected set is now the largest
                if ( cset.size() >= csets[largestSetIdx].size())
                    largestSetIdx = (int)csets.size()-1;
            }   // end if
        }   // end foreach

        // If the expansion front has been exhausted, but there are still connected vertices in cuvidxs
        // we need to add in another connected set.
        if ( setExpandFront.empty() && !cuvidxs.empty())
        {
            csets.resize( csets.size()+1);
            setExpandFront.push_back( *cuvidxs.begin());
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
        BOOST_FOREACH ( const int& cuv, rejectSet)
        {
            if ( _model->getNumSharedFaces( uvidx, cuv) <= 0)   // Dealt with already
                continue;
            const IntSet fids = _model->getSharedFaces( uvidx, cuv);    // Copy out because changing
            BOOST_FOREACH ( const int& fid, fids)
                removeFace( fid);
            removedCount++;
        }   // end foreach
    }   // end for

    return removedCount;
}   // end removeJunctionConnections


/*
int findCandidateFaceTargetVertex( const RFeatures::ObjModelTopologyFinder& modelTopologyFinder,
                                   int workingFace, int ev0, int ev1,
                                   const IntSet& sfaces,
                                   const boost::unordered_map<int,int>& a2pMap)
{
    BOOST_FOREACH ( const int& fid, sfaces)
    {
        if ( fid != workingFace)
        {
            const int uv0 = modelTopologyFinder.getOtherVertex( fid, ev0, ev1);
            if ( a2pMap.count(uv0))
                return uv0;
        }   // end if
    }   // end foreach 
    return -1;
}   // end findCandidateFaceTargetVertex


// private
// Surface joining vertices have some connected vertices that share more than two polygons
// with uvidx and this allows for surface extrusions along a different plane.
int ObjModelCleaner::removeSurfaceJoin( int uvidx)
{
    int ca0 = -1;   // Decided when ca1 and ca2 are both set
    int ca1 = -1;   // Found from looking at edge valpha-uvidx (uvidx will "become" ca1 if found)
    int ca2 = -1;   // Found from looking at edge valpha-cuv (cuv will "become" ca2 if found)
    int f2flip = -1;
    int tcuv = -1;  // Connected vertex that with ca0 specifies the edge we want to rotate f2flip about if found.
    const RFeatures::ObjModelTopologyFinder modelTopologyFinder( _model);

    // Find the connected vertices giving edges that are shared by more than two polys.
    const IntSet& cuvidxs = _model->getConnectedUniqueVertices( uvidx);
    BOOST_FOREACH ( const int& cuv, cuvidxs)
    {   
        if ( _model->getNumSharedFaces( uvidx, cuv) <= 2)
            continue;

        // Find the set of polygons that the edge uvidx-cuv shares.
        boost::unordered_map<int, int> a2pMap;  // alpha (tip) to poly map
        const IntSet& fids = _model->getSharedFaces( uvidx, cuv);
        BOOST_FOREACH ( const int& fid, fids)
        {
            // Get the other vertex (valpha) of this poly that isn't uvidx or cuv.
            const int valpha = modelTopologyFinder.getOtherVertex( fid, uvidx, cuv);
            assert( cuvidxs.count(valpha)); // Every valpha is also in the connected vertices set of uvidx
            a2pMap[valpha] = fid;
        }   // end foreach

        typedef std::pair<int, int> PolyTipPair;
        BOOST_FOREACH ( const PolyTipPair& ptp, a2pMap)
        {
            ca0 = ptp.first;
            const int cf2flip = ptp.second;    // The polygon joined to uvidx-cuv that we're currently looking at

            const IntSet& sf0 = _model->getSharedFaces( ca0, uvidx);
            if ( sf0.size() == 1)
            {
                assert( *sf0.begin() == cf2flip);
                continue;
            }   // end if

            const IntSet& sf1 = _model->getSharedFaces( ca0, cuv);
            if ( sf1.size() == 1)
            {
                assert( *sf1.begin() == cf2flip);
                continue;
            }   // end if

            ca1 = findCandidateFaceTargetVertex( modelTopologyFinder, cf2flip, ca0, uvidx, sf0, a2pMap);
            ca2 = findCandidateFaceTargetVertex( modelTopologyFinder, cf2flip, ca0, cuv,   sf1, a2pMap);
            // If both the "other" vertices are already in the valpha set, we've found the target poly to edit
            if ( ca1 >= 0 && ca2 >= 0)
            {
                f2flip = cf2flip;
                tcuv = cuv;
                break;
            }   // end if
        }   // end foreach

        if ( f2flip >= 0)
            break;
    }   // end foreach

    // The discovered poly (f2flip) is now "rotated" about edge tcuv-ca0 and the polygon
    // vertex reference that was at uvidx is set to refer to the vertex at position ca1. (First choice)
    // Alternatively, the polygon is rotated about edge uvidx-ca0 in which case the polygon
    // vertex reference that was at tcuv is set to refer to the vertex at position ca2.

    // Since uvidx is the "root" unique vertex that this function was called for,
    // our preference is to relocate the polygon by shifting the vertex reference from uvidx
    // to ca1. Only if this isn't possible (i.e. the polygon already exists - unlikely?) do
    // we attempt to move tcuv to ca2. If polygons in both places already exist (super unlikely?),
    // we simply remove the polygon.
    
    int retval = 0;
    if ( f2flip >= 0)
    {
        retval = -1;

        // Test choice 1) by looking at all of the polygons shared about edge tcuv-ca0
        if ( modelTopologyFinder.doesUniquePolyExist( ca0, tcuv, ca1) < 0)
        {
            _model->setFaceFromUnique( ca0, tcuv, ca1);
            retval = 1;
        }   // end if
        else if ( modelTopologyFinder.doesUniquePolyExist( ca0, uvidx, ca2) < 0)
        {
            _model->setFaceFromUnique( ca0, ca2, uvidx);
            retval = 1;
        }   // end else if

        removeFace( f2flip);
        updateUniqueVertexTopology( ca1);
        updateUniqueVertexTopology( ca2);
    }   // end if

    return retval;
}   // end removeSurfaceJoin
*/


int ObjModelCleaner::removeSurfaceJoin( int uvidx)
{
    const RFeatures::ObjModelTopologyFinder modelTopologyFinder( _model);

    // Find the connected vertices giving edges that are shared by more than two polys.
    std::vector<int> remcuvidxs;
    const IntSet& cuvidxs = _model->getConnectedUniqueVertices( uvidx);
    BOOST_FOREACH ( const int& cuv, cuvidxs)
    {   
        if ( _model->getNumSharedFaces( uvidx, cuv) > 2)
            remcuvidxs.push_back(cuv);
    }   // end foreach

    // For these edges uvidx-euv, we remove the shared polygons, then find get all of the
    // polygons that use vertex euv and set them to use uvidx instead. Finally remove euv.
    BOOST_FOREACH ( const int& euv, remcuvidxs)
    {
        if ( _model->getNumSharedFaces( uvidx, euv) <= 2)
            continue;

        const IntSet fids = _model->getSharedFaces( uvidx, euv);    // Copy out because removing
        BOOST_FOREACH ( const int& fid, fids)
            removeFace( fid);   // Remove the shared polygon - may create hole

        // Create new polygons having vertex references of euv instead of uvidx
        const IntSet f2keep = _model->getFaceIdsFromUniqueVertex( uvidx); // Copied out because removing
        BOOST_FOREACH ( const int& fid, f2keep)
        {
            assert( !fids.count(fid));
            // We want to maintain the normal for the face, which means setting the vertices in
            // the correct order. To do this, we retrieve the original polygon and set the new
            // vertices in accordance with the old (except for swapping uvidx for euv of course).
            const ObjPoly& poly = _model->getUniqueVertexFace( fid);
            const int uv0 = poly.vindices[0] == uvidx ? euv : poly.vindices[0];
            const int uv1 = poly.vindices[1] == uvidx ? euv : poly.vindices[1];
            const int uv2 = poly.vindices[2] == uvidx ? euv : poly.vindices[2];
            _model->setFaceFromUnique( uv0, uv1, uv2);
            removeFace( fid);   // Delete the now redundant polygon
        }   // end foreach
    }   // end foreach

    removeVertex( uvidx);
    return 1;
}   // end removeSurfaceJoin


// public
int ObjModelCleaner::remove3D()
{
    int nonFlatEdgeRemovals = 0;
    int junctionARemovals = 0;
    int junctionBRemovals = 0;
    int surfaceJoinRemovals = 0;
    int removedExtrusions = 0;

    const IntSet& uvidxs = _model->getUniqueVertexIds();

    bool loopExtra = true;  // Use one extra loop
    int numNonFlat = (int)_nonflat->size();
    int oldNumNonFlat = numNonFlat > 0 ? numNonFlat+1 : numNonFlat;
    while ( numNonFlat < oldNumNonFlat || loopExtra)
    {
        if ( numNonFlat == oldNumNonFlat)   // One extra loop whenever no change
            loopExtra = false;
        else
            loopExtra = true;

        oldNumNonFlat = numNonFlat;
        removedExtrusions += removeFlatExtrusions();

        IntSet uvtxs = *_nonflat;   // Copy out because changing
        BOOST_FOREACH ( const int& uvidx, uvtxs)
        {
            // Recheck since removal of polys might've affected this vertex.
            if ( _flat->count(uvidx) || !uvidxs.count(uvidx))
                continue;

            if ( _edge->count(uvidx))
            {   // At least one vertex connected to uvidx shares only a single poly with uvidx
                if ( _comp->count(uvidx))
                    nonFlatEdgeRemovals += removeNonFlatMakingEdges( uvidx);
                else
                    junctionARemovals += removeJunctionConnections( uvidx);
            }   // end if
            else
            {   // All vertices connected to uvidx share 2 or more polys with uvidx.
                // This can take two different forms.
                if ( _comp->count(uvidx))
                    surfaceJoinRemovals += removeSurfaceJoin( uvidx);
                else
                    junctionBRemovals += removeJunctionConnections( uvidx);
            }   // end else
        }   // end foreach

        numNonFlat = (int)_nonflat->size();
    }   // end while

    return nonFlatEdgeRemovals + junctionARemovals + junctionBRemovals + surfaceJoinRemovals + removedExtrusions;
}   // end remove3D


// public
int ObjModelCleaner::remove1D()
{
    const IntSet& uvidxs = _model->getUniqueVertexIds();

    int remTotal = 0;
    int nremoved = 0;

    do
    {
        std::vector<int> remSet; // The remove set is edge and flat vertices that aren't complete
        BOOST_FOREACH ( const int& uvidx, *_edge)
        {
            if ( _flat->count(uvidx) && !_comp->count(uvidx))
                remSet.push_back(uvidx);
        }   // end foreach

        nremoved = 0;
        BOOST_FOREACH ( const int& uvidx, remSet)
        {
            if ( !uvidxs.count(uvidx))
                continue;
            nremoved += removeJunctionConnections( uvidx);
        }   // end foreach

        // Removing some faces may have meant that some vertices became lonely,
        // so removal of the lonely vertices happens at the end of this function.
        IntSet uvtxs = *_lonely;   // Copy out because changing
        BOOST_FOREACH ( const int& uvidx, uvtxs)
        {
            if ( removeVertex(uvidx))
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
    const IntSet uvidxs = _model->getUniqueVertexIds(); // Copied out because will be changed!
    BOOST_FOREACH ( const int& ui, uvidxs)
    {
        if ( _model->getUniqueVertexFaceCount(ui) < minVtxFaceConns)
        {
            removeVertexAndFaces( ui);
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

/*
std::vector<double> elens;
const double ulen = RFeatures::calcMeanEdgeDistance( _model, elens);
const double stddev = RFeatures::calcStdDev( elens, ulen, 0);
const double maxLen = ulen + 0.6*stddev;    // Edges greater than this are removed
*/

// public
void ObjModelCleaner::removeSpikes( double maxGrad)
{
    // Collect the spike unique vertices
    std::vector<int> deluvidxs;
    const IntSet& uvidxs = _model->getUniqueVertexIds();
    BOOST_FOREACH ( const int& uvidx, uvidxs)
    {
        if ( _model->getUniqueVertexCurvature( uvidx) > maxGrad)
            deluvidxs.push_back(uvidx);
    }   // end foreach

    BOOST_FOREACH ( const int& uvidx, deluvidxs)
        removeVertexAndFaces( uvidx);
}   // end removeSpikes


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

        const cv::Vec3f nvtx = interpolateVertexFromConnected( _model, uvidx);
        // The new vertex must be sufficiently different from the old one.
        // If it returns the same vertex, we can't replace the vertex.
        const int checkId = _model->lookupUniqueVertexIndex(nvtx);
        if ( checkId == uvidx)
            continue;

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
        removeVertex(uvidx);
}   // end adjustHighGradientVertices


// public
void ObjModelCleaner::removeUniqueVertices( const IntSet& uvidxs)
{
    BOOST_FOREACH ( const int& uvidx, uvidxs)
        removeVertexAndFaces( uvidx);
}   // end removeVertices


/*
// public
int ObjModelCleaner::fillHoles()    // TODO
{
    using RFeatures::ObjModelBoundaryFinder;
    ObjModelBoundaryFinder* ombf = new ObjModelBoundaryFinder( _model);
    const int nbs = ombf->findOrderedBoundaryUniqueVertexIndices(); // Returns number of boundaries
    for ( int i = 0; i < nbs; ++i)
    {
        const std::list<int>& boundary = ombf->getBoundary(i);
    }   // end for
    delete ombf;
    return nbs;
}   // end fillHoles


bool checkDontAdd( const ObjModel::Ptr m, int ui, int minVtxFaceConns, IntSet& vertsNotAdded)
{
    if ( vertsNotAdded.count(ui))
        return true;
    // Don't add the vertex if it has too few face connections, is non-flat, or is a junction vertex.
    if (( m->getUniqueVertexFaceCount(ui) < minVtxFaceConns) || is3DExtrusion( ui))
    {
        vertsNotAdded.insert(ui);
        return true;
    }   // end if
    return false;
}   // end checkDontAdd



// public static
ObjModel::Ptr ObjModelCleaner::createCleanedCopy( const ObjModel::Ptr orig, double maxGrad, int minVtxFaceConns)
{
    ObjModel::Ptr nmodel = ObjModel::create( (int)orig->getSpatialPrecision());

    IntSet facesToAdd;    // Holds indices of faces in orig to add
    IntSet vertsNotAdded;    // Holds indices of unique vertices in orig that aren't added

    boost::unordered_map<int,int> oldVtxsToNewVtxs; // Maps old vertex IDs to the newly added vertex IDs
    const IntSet& uvidxs = orig->getUniqueVertexIds();
    BOOST_FOREACH ( const int& ui, uvidxs)
    {
        if ( checkDontAdd( orig, ui, minVtxFaceConns, vertsNotAdded))
            continue;

        // Okay to add the vertex.
        cv::Vec3f vtx = orig->getUniqueVertex(ui);  // Copied out since may need to change below
        // If this vertex has too high a gradient, adjust it.
        if ( fabs( orig->getUniqueVertexCurvature(ui)) > maxGrad)
        {
            // Note that interpolating may return an essentially unchanged vertex position in some cases.
            vtx = interpolateVertexFromConnected( orig, ui);
        }   // end if

        // Map all the newly added texture vertices to the old ones
        const IntSet& vindices = orig->lookupTextureIndices(ui);
        BOOST_FOREACH ( const int& vid, vindices)
            oldVtxsToNewVtxs[vid] = nmodel->addVertex( vtx, orig->getTextureOffset(vid));

        // Add in the face IDs to add from orig to nmodel
        const IntSet& faceIds = orig->getFaceIdsFromUniqueVertex(ui);
        BOOST_FOREACH ( const int& fid, faceIds)
        {
            // Don't allow the face to be added if any of its vertices shouldn't be added
            const ObjPoly& uface = orig->getUniqueVertexFace(fid);
            if ( checkDontAdd( orig, uface.vindices[0], minVtxFaceConns, vertsNotAdded) ||
                 checkDontAdd( orig, uface.vindices[1], minVtxFaceConns, vertsNotAdded) ||
                 checkDontAdd( orig, uface.vindices[2], minVtxFaceConns, vertsNotAdded))
                continue;

            facesToAdd.insert(fid);
        }   // end foreach
    }   // end foreach

    // Add in the faces
    int v0, v1, v2;
    BOOST_FOREACH ( const int& fid, facesToAdd)
    {
        const ObjPoly& oface = orig->getFace(fid);
        v0 = oface.vindices[0]; v1 = oface.vindices[1]; v2 = oface.vindices[2];
        nmodel->setFace( oldVtxsToNewVtxs.at(v0), oldVtxsToNewVtxs.at(v1), oldVtxsToNewVtxs.at(v2));
    }   // end foreach

    nmodel->setTexture( orig->getTexture());
    return nmodel;
}   // end createCleanedCopy
*/
