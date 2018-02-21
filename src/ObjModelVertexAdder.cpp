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

#include <ObjModelVertexAdder.h>
#include <ObjModelPolygonAreaCalculator.h>
#include <boost/heap/fibonacci_heap.hpp>
#include <boost/foreach.hpp>
#include <cassert>
#include <queue>
using RFeatures::ObjModelVertexAdder;
using RFeatures::ObjModel;

namespace
{

void checkAddLargeTriangles( std::queue<int>& fids, const IntSet& fset, const ObjModel::Ptr model, double maxTriangleArea)
{
    BOOST_FOREACH ( int fid, fset)
    {
        if ( RFeatures::ObjModelPolygonAreaCalculator::calcFaceArea( model, fid) > maxTriangleArea)
            fids.push(fid);
    }   // end foreach
}   // end checkAddLargeTriangles

}   // end namespace


// public
ObjModelVertexAdder::ObjModelVertexAdder( ObjModel::Ptr model) : _model(model)
{
}   // end ctor
                                                        

// public
int ObjModelVertexAdder::addVerticesToMaxTriangleArea( double maxTriangleArea)
{
    std::queue<int> fids;
    checkAddLargeTriangles( fids, _model->getFaceIds(), _model, maxTriangleArea);

    int nadded = 0;
    while ( !fids.empty())
    {
        const int fid = fids.front();
        fids.pop();

        // Subdivision position as mean of vertices
        const int* vidxs = _model->getFaceVertices(fid);
        const cv::Vec3f npos = (_model->vtx(vidxs[0]) + _model->vtx(vidxs[1]) + _model->vtx(vidxs[2])) * 1.0f/3;
        int nvidx = _model->subDivideFace( fid, npos);
        // Check the newly created subdivided faces to see if they need to be subdivided also...
        checkAddLargeTriangles( fids, _model->getFaceIds(nvidx), _model, maxTriangleArea);
        nadded++;
    }   // end while

    return nadded;
}   // end addVerticesToMaxTriangleArea


/*
namespace
{

struct FaceArea;

struct FaceAreaComparator
{
    bool operator()( const FaceArea* fa0, const FaceArea* fa1) const;
};  // end struct

typedef boost::heap::fibonacci_heap<FaceArea*, boost::heap::compare<FaceAreaComparator> > FaceAreaQueue;
typedef FaceAreaQueue::handle_type QHandle;

struct FaceArea
{
    int _fid;
    double _area;   // Face area
    QHandle _qhandle;

    FaceArea( int fid, double area) : _fid(fid), _area(area) {}
};  // end struct

bool FaceAreaComparator::operator()( const FaceArea* fa0, const FaceArea* fa1) const
{
    return fa0->_area <= fa1->_area;
}   // end operator()


class ModelSubdivideAndMerge
{
public:
    ModelSubdivideAndMerge( ObjModel::Ptr m) : _model(m) {}

    int operator()( double maxTriangleArea, int maxDebugIterations)
    {
        const IntSet& fids = _model->getFaceIds();
        BOOST_FOREACH ( int fid, fids)
            addToQueue(fid, maxTriangleArea);

        std::cerr << "maxTriangleArea = " << maxTriangleArea << std::endl;
        int vi, vj;
        int nvidx;
        int nadded = 0;
        int numDebugIterations = 0; // DEBUG
        while ( numDebugIterations < maxDebugIterations)
        {
            numDebugIterations++;   // DEBUG
            FaceArea* fa = pop();
            int fid = fa->_fid;
            std::cerr << "ObjModelVertexAdder::subdivideAndMerge: On face " << fid << " with area " << fa->_area << std::endl;
            if ( fa->_area <= maxTriangleArea)
                break;

            delete fa;

            //_model->showDebug(true);

            // Subdivision position as mean of vertices
            const int* vidxs = _model->getFaceVertices(fid);
            assert( vidxs != NULL);
            const cv::Vec3f npos = (_model->vtx(vidxs[0]) + _model->vtx(vidxs[1]) + _model->vtx(vidxs[2])) * 1.0f/3;
            nvidx = _model->subDivideFace( fid, npos);
            nadded++;   // New vertex added

            // The newly added face IDs (copied out because flipFacePair can change vertex face membership
            const IntSet nfids = _model->getFaceIds(nvidx);
            BOOST_FOREACH ( int nfid, nfids)    // Flip edge orientation of adjacent faces
            {
                _model->poly(nfid).getOpposite( nvidx, vi, vj);
                if ( _model->getNumSharedFaces( vi, vj) > 1)
                    _model->flipFacePair( vi, vj);
            }   // end foreach

            // Update the priority queue with the updated areas of the faces connected to nvidx.
            BOOST_FOREACH ( int nfid, _model->getFaceIds(nvidx))
            {
                if ( _fareas.count(nfid) == 0)
                    addToQueue( nfid, maxTriangleArea);
            }   // end foreach

            BOOST_FOREACH ( int nfid, _model->getFaceIds(nvidx))
            {
                FaceArea* fa2 = _fareas.at(nfid);
                fa2->_area = RFeatures::ObjModelPolygonAreaCalculator::calcFaceArea( _model, nfid);
                _queue.decrease( fa2->_qhandle);   // O(log(N))
            }   // end foreach
        }   // end while

        while ( !_queue.empty())
        {
            FaceArea* fa = pop();
            std::cerr << fa->_area << std::endl;
            delete fa;
        }   // end while

        return nadded;
    }   // end operator()

private:
    ObjModel::Ptr _model;
    FaceAreaQueue _queue;
    boost::unordered_map<int, FaceArea*> _fareas;

    void addToQueue( int fid, double maxTriangleArea)
    {
        const double area = RFeatures::ObjModelPolygonAreaCalculator::calcFaceArea( _model, fid);
        if ( area <= maxTriangleArea)   // Don't add if not too large!
            return;
        FaceArea* fa = new FaceArea( fid, area);
        fa->_qhandle = _queue.push(fa);
        _fareas[fid] = fa;
    }   // end addToQueue

    FaceArea* pop()
    {
        FaceArea* fa = _queue.top();
        _queue.pop();
        _fareas.erase(fa->_fid);
        return fa;
    }   // end pop
};  // end class


}   // end namespace
*/


// public
int ObjModelVertexAdder::subdivideAndMerge( double maxTriangleArea)
{
    // Find the set of faces with area greater than maxTriangleArea
    IntSet* fset = new IntSet;
    const IntSet& fids = _model->getFaceIds();
    BOOST_FOREACH ( int fid, fids)
    {
        const double area = RFeatures::ObjModelPolygonAreaCalculator::calcFaceArea( _model, fid);
        if ( area > maxTriangleArea)
            fset->insert(fid);
    }   // end foreach

    int nadded = 0;

    boost::unordered_set<RFeatures::Edge, RFeatures::HashEdge> *fedges = new boost::unordered_set<RFeatures::Edge, RFeatures::HashEdge>;
    IntSet* mset = new IntSet;
    IntSet* bset = new IntSet;
    while ( !fset->empty()) // While no more faces larger than the maximum triangle area...
    {
        while ( !fset->empty()) // While no more faces larger than the maximum triangle area...
        {
            int fid = *fset->begin();
            fset->erase(fid);
            // Get the set of edges that may be flipped from this triangle
            // (includes edges not sharing a single pair of texture coordinates).
            const int* vidxs = _model->getFaceVertices(fid);
            fedges->insert( RFeatures::Edge( vidxs[0], vidxs[1]));
            fedges->insert( RFeatures::Edge( vidxs[0], vidxs[2]));
            fedges->insert( RFeatures::Edge( vidxs[1], vidxs[2]));

            // Subdivide this face into three new polygons with new vertex at the centre.
            const cv::Vec3f npos = (_model->vtx(vidxs[0]) + _model->vtx(vidxs[1]) + _model->vtx(vidxs[2])) * 1.0f/3;
            int nvidx = _model->subDivideFace( fid, npos);
            nadded++;   // New vertex added

            // Add the triangles that may partake in edge flipping (these are the
            // newly subdivided triangles with nvidx as a member vertex).
            BOOST_FOREACH ( int nfid, _model->getFaceIds(nvidx))
                mset->insert(nfid); // Triangles that may possibly be edge flipped
        }   // end while

        BOOST_FOREACH ( const RFeatures::Edge& edge, *fedges)
        {
            const int v0 = edge.v0;
            const int v1 = edge.v1;
            const IntSet& sfids = _model->getSharedFaces( v0, v1);
            // If there are exactly two shared faces and they're both in the mset, flip the adjoining edge.
            if ( sfids.size() == 2)
            {
                int f0 = *sfids.begin();
                int f1 = *(++sfids.begin());
                if ( mset->count(f0) > 0 && mset->count(f1) > 0)
                {
                    // If 1-to-1 mapping of geometry edge to texture edge, then just flip the face pair
                    // normally. However, if there's a larger number of texture edges mapped to the
                    // geometry edge, need to introduce a new vertex which adds new faces!
                    if ( _model->getNumTextureEdges( v0, v1) <= 1)
                    {
                        // Only flip if the new edge will be shorter than the existing one.
                        int v2 = _model->poly(f0).getOpposite( v0, v1);
                        int v3 = _model->poly(f1).getOpposite( v0, v1);
                        if ( cv::norm( _model->vtx(v2) - _model->vtx(v3)) < cv::norm( _model->vtx(v0) - _model->vtx(v1)))
                            _model->flipFacePair( v0, v1);
                    }   // end if
                    else
                    {
                        // New vertex exactly midway between the edge vertices
                        const cv::Vec3f npos = (_model->vtx(v0) + _model->vtx(v1)) * 0.5f;
                        const int nvidx = _model->addVertex( npos);
                        _model->subDivideEdge( v0, v1, nvidx);
                        nadded++;
                        BOOST_FOREACH ( int nfid, _model->getFaceIds(nvidx))
                            mset->insert(nfid);
                    }   // end else
                }   // end if
            }   // end if
        }   // end foreach

        BOOST_FOREACH ( int fid, *mset)
        {
            if ( RFeatures::ObjModelPolygonAreaCalculator::calcFaceArea( _model, fid) > maxTriangleArea)
                bset->insert( fid);
        }   // end foreach

        fedges->clear();
        mset->clear();

        IntSet* tmp = fset;
        fset = bset;
        bset = tmp;
    }   // end while

    delete fset;
    delete bset;
    delete mset;
    delete fedges;

    return nadded;
}   // end subdivideAndMerge


namespace
{

typedef boost::unordered_map<int,int> IntIntMap;
typedef std::pair<int,int> IPair;

struct ModelEdgeUpdater
{
    ModelEdgeUpdater( ObjModel::Ptr m) : _model(m) {}

    // Update membership of _f1, _f2, and _f3 based on existing membership where eid is ID of edge that's too long.
    void updateFaceMembership( int fid, int eid)
    {
        if ( _f1.count(fid) > 0)
        {
            int eid2 = _f1.at(fid); // The other edge that's presumed to be too long.
            assert( eid != eid2);
            _f1.erase(fid);
            // Get the edge from face fid that isn't eid or eid2
            int feids[3];
            getFaceEdgeIDs( fid, feids);
            for ( int i = 0; i < 3; ++i)
            {
                if ( feids[i] != eid && feids[i] != eid2)
                {
                    _f2[fid] = feids[i];
                    break;
                }   // end if
            }   // end for
        }   // end if
        else if ( _f2.count(fid) > 0)
        {
            _f3.insert(fid);
            _f2.erase(fid);
            assert( _f1.count(fid) == 0);
        }   // end else if
        else if ( _f3.count(fid) == 0)
            _f1[fid] = eid;
    }   // end updateFaceMembership


    double getEdgeLen( int eid)
    {
        int v0, v1;
        _model->getEdge( eid, v0, v1);
        return cv::norm( _model->vtx(v0) - _model->vtx(v1));
    }   // end getEdgeLen


    // Set eids with the edge IDs of face fid returning true if fid is legal.
    bool getFaceEdgeIDs( int fid, int* eids)
    {
        const int* vidxs = _model->getFaceVertices(fid);
        if ( !vidxs)
            return false;
        eids[0] = _model->getEdgeId(vidxs[0], vidxs[1]);
        eids[1] = _model->getEdgeId(vidxs[1], vidxs[2]);
        eids[2] = _model->getEdgeId(vidxs[2], vidxs[0]);
        return true;
    }   // end getFaceEdgeIDs


    ObjModel::Ptr _model;
    IntIntMap _f1;  // Face IDs map to the edge that's too long.
    IntIntMap _f2;  // Face IDs map to the edge that's NOT too long.
    IntSet _f3;     // All edges treated as too long.
};  // end struct

}   // end namespace


// public
void ObjModelVertexAdder::subdivideEdges( double maxEdgeLen)
{
    ModelEdgeUpdater meu(_model);

    IntSet *eset = new IntSet;    // Set of all candidate edges to parse (initially, need to check all model edges)
    BOOST_FOREACH ( int eid, _model->getEdgeIds())
    {
        if ( meu.getEdgeLen(eid) > maxEdgeLen)
            eset->insert(eid);
    }   // end foreach

    //int nits = 0;
    while ( !eset->empty())
    {
        /*
        std::cerr << "\n   ***** Start of subdivision iteration " << nits << std::endl;
        _model->showDebug(true);
        nits++;
        */
        // Set the initial memberships of f1,f2,f3 for all faces associated with edges in eset.
        // Faces in f1 have only a single edge that's too long.
        // Faces in f2 have two edges that are too long.
        // Faces in f3 have all three of their edges too long.
        BOOST_FOREACH ( int eid, *eset)
        {
            BOOST_FOREACH ( int fid, _model->getSharedFaces(eid))
                meu.updateFaceMembership( fid, eid);
        }   // end foreach
        eset->clear();

        // For all faces in f2 (having two edges too long), this will mean treating the other edge
        // as a pseudo long edge since the face must be subdivided. This means finding the adjacent
        // face(s) to this psuedo edge and updating the membership of the associated face(s).
        while ( !meu._f2.empty())
        {
            int eid = meu._f2.begin()->second;
            BOOST_FOREACH ( int fa, _model->getSharedFaces(eid))
                meu.updateFaceMembership( fa, eid);
        }   // end while

        /*
        std::cerr << meu._f1.size() << " faces in f1" << std::endl;
        std::cerr << meu._f2.size() << " faces in f2" << std::endl;
        std::cerr << meu._f3.size() << " faces in f3" << std::endl;
        */

        // _f2 now empty, faces in _f3 require subdivision, and faces in _f1 require subdivision along an edge.
        // Create new midpoint vertices for all edges of faces in _f1. Some of these vertices will be referenced
        // when attempting to add new vertices in the same positions due to a face in _f1 being adjacent to
        // an existing face in _f3. This should be okay as long as the vertex lookup functionality in ObjModel works!
        IntIntMap e2nvs;
        int v0, v1;
        BOOST_FOREACH ( const IPair& ip, meu._f1)
        {
            _model->getEdge( ip.second, v0, v1);
            const cv::Vec3f nv = (_model->vtx(v0) + _model->vtx(v1)) * 0.5f;
            e2nvs[ip.second] = _model->addVertex( nv);
        }   // end foreach
        meu._f1.clear();
     
        BOOST_FOREACH ( int fid, meu._f3) 
        {
            int nfids[4];   // Storage for IDs of faces added due to subdivision of face fid
            int nfid = _model->subDivideFace( fid, nfids); // Do subdivision (breaks meshing with currently connected faces)

            // Test the new edges to see if their lengths are > maxEdgeLen for the next iteration.
            int feids[3];
            for ( int i = 1; i < 4; ++i)
            {
                meu.getFaceEdgeIDs( nfids[i], feids);
                if ( meu.getEdgeLen(feids[0]) > maxEdgeLen)
                    eset->insert(feids[0]);
                if ( meu.getEdgeLen(feids[1]) > maxEdgeLen)
                    eset->insert(feids[1]);
                if ( meu.getEdgeLen(feids[2]) > maxEdgeLen)
                    eset->insert(feids[2]);
            }   // end for
        }   // end foreach
        meu._f3.clear();

        // Do the edge subdivisions
        BOOST_FOREACH ( const IPair& ip, e2nvs)
        {
            bool foundEdge = _model->getEdge( ip.first, v0, v1);
            assert(foundEdge);
            foundEdge = _model->subDivideEdge( v0, v1, ip.second);
            assert(foundEdge);
            // Don't need to check the new edge created to see if it's > maxEdgeLen, because the other two
            // edges of the triangle originally having eid as an edge weren't found to be too long, so
            // geometrically, the new edge cannot be longer than both of these edges.
        }   // end foreach
    }   // end while

    //std::cerr << "\nFINISHED SUBDIVISION\n" << std::endl;
    delete eset;
}   // end subdivideEdges
