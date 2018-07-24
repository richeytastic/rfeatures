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

#include <ObjModelNormals.h>
#include <ObjModelHoleFiller.h>
#include <FeatureUtils.h>
#include <cassert>
#include <boost/heap/fibonacci_heap.hpp>
using RFeatures::ObjModelTriangleMeshParser;
using RFeatures::ObjModelHoleFiller;
using RFeatures::ObjModel;

namespace {
struct InnerAngle;

struct InnerAngleComparator { bool operator()( const InnerAngle*, const InnerAngle*) const; };

typedef boost::heap::fibonacci_heap<InnerAngle*, boost::heap::compare<InnerAngleComparator> > InnerAngleQueue;
typedef InnerAngleQueue::handle_type QHandle;

struct InnerAngle
{
    int _prev;
    int _vidx;   // Vertex ID
    int _next;
    double _angle;  // Inner angle value (higher --> more acute)
    QHandle _qhandle;

    InnerAngle( int i, int j, int k, double a) : _prev(i), _vidx(j), _next(k), _angle(a) {}
};  // end struct

bool InnerAngleComparator::operator()( const InnerAngle* ia0, const InnerAngle* ia1) const
{
    //return ia0->_angle >= ia1->_angle;
    return ia0->_angle <= ia1->_angle;
}   // end operator()



struct FillHoleHelper
{
    ObjModel::Ptr _model;
    InnerAngleQueue _queue;
    std::unordered_map<int, InnerAngle*> _iangles;


    FillHoleHelper( ObjModel::Ptr m, const std::list<int>& blist) : _model(m)
    {
        // Place all vertices into priority queue
        std::list<int>::const_iterator last = --blist.end();
        for ( std::list<int>::const_iterator i = blist.begin(); i != blist.end(); ++i)
        {
            std::list<int>::const_iterator j = i;
            if ( i == last)
                j = blist.begin();
            else
                j++;

            std::list<int>::const_iterator k = j;
            if ( j == last)
               k = blist.begin();
            else
                k++;

            addToQueue( *i, *j, *k);
        }   // end for
    }   // end ctor


    ~FillHoleHelper()
    {
        while ( !_queue.empty())
            delete pop();
    }   // end dtor


    void fillHole( IntSet *newPolys=nullptr)
    {
        while ( _queue.size() > 3)
        {
            // The set of not allowed vertices includes the vertices of new edges.
            std::unordered_map<int, InnerAngle*> disallow;
            while ( !_queue.empty())
            {
                InnerAngle* ia = pop();
                if ( disallow.count( ia->_vidx) > 0)
                    continue;

                if ( !allowEdgePairParse(ia))
                {
                    disallow[ia->_vidx] = ia;
                    continue;
                }   // end if

                _model->setEdge( ia->_prev, ia->_next); // Set the edge
                if ( newPolys)  // Record the polys associated with this new edge
                {
                    const IntSet& sfids = _model->getSharedFaces( ia->_prev, ia->_next);
                    newPolys->insert( sfids.begin(), sfids.end());
                }   // end if

                // Update prev and next vertex adjacent vertex refs
                InnerAngle* ip = _iangles.at(ia->_prev);
                InnerAngle* in = _iangles.at(ia->_next);
                delete ia;
                ip->_next = in->_vidx;
                in->_prev = ip->_vidx;

                // Ensure these vertices can't be set on this iteration
                disallow[ip->_vidx] = ip;
                disallow[in->_vidx] = in;
            }   // end while

            for ( const auto& iapair : disallow)
            {
                InnerAngle* ia = iapair.second;
                ia->_angle = calcInnerAngle( ia->_prev, ia->_vidx, ia->_next);
                ia->_qhandle = _queue.push(ia);
            }   // end foreach
        }   // end while
    }   // end fillHole


    void addToQueue( int vi, int vj, int vk)
    {
        const double ang = calcInnerAngle(vi,vj,vk);    // Inner angle calculated at j
        InnerAngle* ia = new InnerAngle( vi, vj, vk, ang);
        ia->_qhandle = _queue.push(ia);
        _iangles[vj] = ia;
    }   // end addToQueue


    void calcEdgeNormVectors( int i, int j, int k, cv::Vec3f& e0, cv::Vec3f& e1)
    {
        const cv::Vec3f& vi = _model->vtx(i);
        const cv::Vec3f& vj = _model->vtx(j);
        const cv::Vec3f& vk = _model->vtx(k);
        cv::normalize( vi - vj, e0);
        cv::normalize( vk - vj, e1);
    }   // end calcEdgeNormVectors


    double calcInnerAngle( int i, int j, int k)
    {
        cv::Vec3f e0, e1;
        calcEdgeNormVectors( i, j, k, e0, e1);
        return e0.dot(e1);  // [-1,1]
    }   // end calcInnerAngle


    bool allowEdgePairParse( const InnerAngle* ia)
    {
        // Don't parse an edge pair if the edges belong to the same triangle.
        if ( _model->getFaceIds( ia->_vidx).size() == 1)
            return false;

        const cv::Vec3f& bv0 = _model->vtx(ia->_vidx);

        // Find point tv as half way between the normal vectors along the edges rooted at bv0
        cv::Vec3f e0, e1;
        calcEdgeNormVectors( ia->_prev, ia->_vidx, ia->_next, e0, e1);
        const cv::Vec3f tv = 0.5f * (e0 + e1) + bv0;

        // Do the same, but for the edges opposite to the candidate parse edges.
        int f0 = *_model->getSharedFaces( ia->_prev, ia->_vidx).begin();
        int f1 = *_model->getSharedFaces( ia->_next, ia->_vidx).begin();
        int i0 = _model->poly(f0).getOpposite( ia->_prev, ia->_vidx);
        int i1 = _model->poly(f1).getOpposite( ia->_next, ia->_vidx);
        calcEdgeNormVectors( i0, ia->_vidx, i1, e0, e1);
        const cv::Vec3f bv1 = 0.5f * (e0 + e1) + bv0;

        const cv::Vec3f va = tv - bv0;
        const cv::Vec3f vb = tv - bv1;
        // If the dot product of va and vb is positive, then this is a valid edge pair candidate for a new triangle.
        return va.dot(vb) > 0;
    }   // end allowEdgePairParse


    InnerAngle* pop()
    {
        InnerAngle* ia = _queue.top();
        _queue.pop();
        return ia;
    }   // end pop
};  // end class


// Check if the given list of vertices is connected in sequence with the first connected to the last.
bool checkConnected( const ObjModel::Ptr m, const std::list<int>& blist)
{
    std::list<int>::const_iterator p = --blist.end();   // End of list
    std::list<int>::const_iterator n = blist.begin();   // Start of list
    while ( n != blist.end())
    {
        if ( m->getNumSharedFaces(*p, *n) != 1)
            return false;
        p = n;
        n++;
    }   // end while
    return true;
}   // end checkConnected

}   // end namespace


ObjModelHoleFiller::ObjModelHoleFiller( ObjModel::Ptr m) : _model(m) {}


void ObjModelHoleFiller::fillHole( const std::list<int>& blist, IntSet *newPolys)
{
    assert( checkConnected( _model, blist));

    // Special case for blist.size() == 3 (just need to simply set a triangle)
    if ( blist.size() == 3)
    {
        std::list<int>::const_iterator it = blist.begin();
        const int vi = *it++;
        const int vj = *it++;
        const int vk = *it++;
        int fid = _model->setFace( vi, vj, vk);
        if ( newPolys)
            newPolys->insert(fid);
    }   // end if
    else
    {
        FillHoleHelper helper( _model, blist);
        helper.fillHole( newPolys);
    }   // end else
}   // end fillHole