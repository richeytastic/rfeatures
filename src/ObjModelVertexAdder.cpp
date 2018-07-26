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
#include <ObjModelPolygonAreas.h>
#include <boost/heap/fibonacci_heap.hpp>
#include <cassert>
#include <queue>
using RFeatures::ObjModelVertexAdder;
using RFeatures::ObjModel;
using std::unordered_map;
using std::unordered_set;

namespace {

void checkAddLargeTriangles( std::queue<int>& fids, const IntSet& fset, const ObjModel* model, double maxTriangleArea)
{
    for ( int fid : fset)
    {
        if ( RFeatures::ObjModelPolygonAreas::calcFaceArea( model, fid) > maxTriangleArea)
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
    checkAddLargeTriangles( fids, _model->getFaceIds(), _model.get(), maxTriangleArea);

    static const float ONE_THIRD = 1.0f/3;
    int nadded = 0;
    while ( !fids.empty())
    {
        const int fid = fids.front();
        fids.pop();

        // Subdivision position as mean of vertices
        const int* vidxs = _model->getFaceVertices(fid);
        const cv::Vec3f npos = (_model->vtx(vidxs[0]) + _model->vtx(vidxs[1]) + _model->vtx(vidxs[2])) * ONE_THIRD;
        int nvidx = _model->subDivideFace( fid, npos);

        // Check the newly created subdivided faces to see if they need to be subdivided also...
        checkAddLargeTriangles( fids, _model->getFaceIds(nvidx), _model.get(), maxTriangleArea);
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
        for ( int fid : fids)
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
            for ( int nfid : nfids)    // Flip edge orientation of adjacent faces
            {
                _model->poly(nfid).getOpposite( nvidx, vi, vj);
                if ( _model->getNumSharedFaces( vi, vj) > 1)
                    _model->flipFacePair( vi, vj);
            }   // end foreach

            // Update the priority queue with the updated areas of the faces connected to nvidx.
            for ( int nfid : _model->getFaceIds(nvidx))
            {
                if ( _fareas.count(nfid) == 0)
                    addToQueue( nfid, maxTriangleArea);
            }   // end foreach

            for ( int nfid : _model->getFaceIds(nvidx))
            {
                FaceArea* fa2 = _fareas.at(nfid);
                fa2->_area = RFeatures::ObjModelPolygonAreas::calcFaceArea( _model, nfid);
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
    unordered_map<int, FaceArea*> _fareas;

    void addToQueue( int fid, double maxTriangleArea)
    {
        const double area = RFeatures::ObjModelPolygonAreas::calcFaceArea( _model, fid);
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
    for ( int fid : fids)
    {
        const double area = RFeatures::ObjModelPolygonAreas::calcFaceArea( _model.get(), fid);
        if ( area > maxTriangleArea)
            fset->insert(fid);
    }   // end foreach

    int nadded = 0;

    unordered_set<RFeatures::Edge, RFeatures::HashEdge> *fedges = new unordered_set<RFeatures::Edge, RFeatures::HashEdge>;
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
            fedges->insert( RFeatures::Edge( vidxs[1], vidxs[2]));
            fedges->insert( RFeatures::Edge( vidxs[2], vidxs[0]));

            // Subdivide this face into three new polygons with new vertex at the centre.
            const cv::Vec3f npos = (_model->vtx(vidxs[0]) + _model->vtx(vidxs[1]) + _model->vtx(vidxs[2])) * 1.0f/3;
            int nvidx = _model->subDivideFace( fid, npos);
            nadded++;   // New vertex added

            // Add the triangles that may partake in edge flipping (these are the
            // newly subdivided triangles with nvidx as a member vertex).
            for ( int nfid : _model->getFaceIds(nvidx))
                mset->insert(nfid); // Triangles that may possibly be edge flipped
        }   // end while

        for ( const RFeatures::Edge& edge : *fedges)
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
                        for ( int nfid : _model->getFaceIds(nvidx))
                            mset->insert(nfid);
                    }   // end else
                }   // end if
            }   // end if
        }   // end foreach

        for ( int fid : *mset)
        {
            if ( RFeatures::ObjModelPolygonAreas::calcFaceArea( _model.get(), fid) > maxTriangleArea)
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
