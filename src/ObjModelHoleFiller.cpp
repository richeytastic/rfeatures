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

struct InnerAngleComparator
{
    bool operator()( const InnerAngle* ia0, const InnerAngle* ia1) const;
};  // end struct

typedef boost::heap::fibonacci_heap<InnerAngle*, boost::heap::compare<InnerAngleComparator> > InnerAngleQueue;
typedef InnerAngleQueue::handle_type QHandle;

struct InnerAngle
{
    int _prev;
    int _vidx;   // Vertex ID
    int _next;
    double _angle;  // Inner angle value (higher --> more acute)
    QHandle _qhandle;

    InnerAngle( int i, int j, int k, double ia) : _prev(i), _vidx(j), _next(k), _angle(ia) {}
};  // end struct

bool InnerAngleComparator::operator()( const InnerAngle* ia0, const InnerAngle* ia1) const
{
    return ia0->_angle >= ia1->_angle;
    //return ia0->_angle <= ia1->_angle;
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


    void fillHole( IntSet *newPolys=NULL)
    {
        while ( _queue.size() > 3)
        {
            // The set of not allowed vertices includes the vertices of new edges.
            std::unordered_map<int, InnerAngle*> notAllowed;
            size_t numNeg = 0;
            while ( (notAllowed.size() + _queue.size()) > 3 && !_queue.empty())
            {
                InnerAngle* ia = pop();
                if ( notAllowed.count( ia->_vidx) == 0)  // Is this vertex to be ignored in this iteration?
                {
                    if ( ia->_angle < 0.0)
                    {
                        notAllowed[ia->_vidx] = ia;
                        numNeg++;
                    }   // end if
                    else
                    {
                        int edgeId = _model->setEdge( ia->_prev, ia->_next); // Set the edge
                        if ( newPolys)  // Record the polys associated with this new edge
                        {
                            const IntSet& sfids = _model->getSharedFaces(edgeId);
                            newPolys->insert( sfids.begin(), sfids.end());
                        }   // end if

                        // Update prev and next vertex adjacent vertex refs
                        InnerAngle* ip = _iangles.at(ia->_prev);
                        InnerAngle* in = _iangles.at(ia->_next);
                        delete ia;
                        ip->_next = in->_vidx;
                        in->_prev = ip->_vidx;

                        // Ensure these vertices can't be set on this iteration
                        notAllowed[ip->_vidx] = ip;
                        notAllowed[in->_vidx] = in;
                    }   // end else
                }   // end if
            }   // end while

            if ( _queue.empty())
            {
                const double amult = numNeg == notAllowed.size() ? -1.0 : 1.0;
                typedef std::pair<int, InnerAngle*> IAPair;
                for ( const IAPair& iapair : notAllowed)
                {
                    InnerAngle* ia = iapair.second;
                    ia->_angle = amult * calcInnerAngle( ia->_prev, ia->_vidx, ia->_next);
                    ia->_qhandle = _queue.push(ia);
                }   // end foreach
            }   // end if
        }   // end while
    }   // end fillHole


    double addToQueue( int vi, int vj, int vk)
    {
        const double ang = calcInnerAngle(vi,vj,vk);
        InnerAngle* ia = new InnerAngle( vi, vj, vk, ang);
        ia->_qhandle = _queue.push(ia);
        _iangles[vj] = ia;
        return ang;
    }   // end addToQueue


    InnerAngle* pop()
    {
        InnerAngle* ia = _queue.top();
        _queue.pop();
        //_iangles.erase(ia->_vidx);
        return ia;
    }   // end pop


    double calcInnerAngle( int i, int j, int k)
    {
        const cv::Vec3f& vi = _model->vtx(i);
        const cv::Vec3f& vj = _model->vtx(j);
        const cv::Vec3f& vk = _model->vtx(k);

        const cv::Vec3f e0 = vi - vj;
        const cv::Vec3f e1 = vk - vj;

        double ia = 0.0;
        if ( cv::norm( vi-vk) < (cv::norm(e0) + cv::norm(e1))) // Perfectly straight
        {
            const int fij = *_model->getSharedFaces( i,j).begin();   // Polygon ID adjacent to e0
            const int x = _model->poly(fij).getOpposite( i,j);
            const cv::Vec3f fnorm = RFeatures::ObjModelNormals::calcNormal( _model, x, i, j);
            const cv::Vec3d enorm = e0.cross(e1);
            // If these norms point in into the same half of the 3D space
            // (+ve dot product) the inner angle at j is positive.
            ia = enorm.dot(fnorm);
        }   // end if
        return ia;
    }   // end calcInnerAngle
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
