/************************************************************************
 * Copyright (C) 2019 Richard Palmer
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
#include <boost/heap/fibonacci_heap.hpp>
#include <cassert>
using RFeatures::ObjModelTriangleMeshParser;
using RFeatures::ObjModelHoleFiller;
using RFeatures::ObjModel;
using RFeatures::ObjPoly;

namespace {

#ifndef NDEBUG
// Check if the given list of vertices is connected in sequence with the first connected to the last.
bool checkConnected( const ObjModel::Ptr model, const std::list<int>& blist)
{
    std::list<int>::const_iterator p = --blist.end();   // End of list
    std::list<int>::const_iterator n = blist.begin();   // Start of list
    while ( n != blist.end())
    {
        if ( !model->hasEdge( *p, *n))  // TODO This should strictly be an edge between triangles only in the provided manifold
            return false;
        p = n;
        n++;
    }   // end while
    return true;
}   // end checkConnected
#endif


double calcArea( const ObjModel* model, int i, int j, int k)
{
    const cv::Vec3f& vi = model->vtx(i);
    const cv::Vec3f& vj = model->vtx(j);
    const cv::Vec3f& vk = model->vtx(k);
    return RFeatures::calcTriangleArea( vi, vj, vk);
}   // end calcArea

int nextFace( const ObjModel* model, const IntSet& mpolys, const IntSet& npolys, int ft, int vi, int vj)
{
    for ( int f : model->spolys(vi,vj))
    {
        if ( (mpolys.count(f) > 0 || npolys.count(f) > 0) && f != ft)
            return f;
    }   // end for
    return -1;
}   // end nextFace


cv::Vec3f calcVertexNorm( const ObjModel* model, const IntSet& mpolys, const IntSet& npolys, int vi, int vj)
{
    IntSet lfids;
    lfids.insert(-1);

    cv::Vec3f vjn( 0, 0, 0);
    int fid = nextFace( model, mpolys, npolys, -1, vi, vj); // Get the face in the manifold starting at edge (_prev, _vidx)
    assert( fid >= 0);
    while ( lfids.count(fid) == 0)
    {
        lfids.insert(fid);  // So we don't parse polygon fid again.
        //std::cerr << "fid = " << fid << std::endl;
        int vk = model->face(fid).opposite(vi,vj); // Get opposite edge vertex for next edge vi,vj.
        // Go around the edges of the connected triangles setting the weighted norms. Calculate based
        // on the known order of vertices around pivot vertex vj (don't use model->calcFaceNorm!)
        cv::Vec3f vik;
        cv::normalize( (model->vtx(vi) - model->vtx(vj)).cross(model->vtx(vk) - model->vtx(vj)), vik);
        vjn += vik * static_cast<float>(calcArea( model, vi, vj, vk)); // Add weighted face norm to vertex norm.
        fid = nextFace( model, mpolys, npolys, fid, vi = vk, vj);
    }   // end while
    return vjn;
}   // end calcVertexNorm


struct InnerAngle;

struct InnerAngleComparator { bool operator()( const InnerAngle*, const InnerAngle*) const; };

typedef boost::heap::fibonacci_heap<InnerAngle*, boost::heap::compare<InnerAngleComparator> > InnerAngleQueue;
typedef InnerAngleQueue::handle_type QHandle;

struct InnerAngle
{
    int _prev;
    int _vidx;   // Pivot vertex
    int _next;
    QHandle _qhandle;

    void setNext( int v, const ObjModel* model, const IntSet& mpolys, const IntSet& npolys)
    {
        _next = v;
        update( model, mpolys, npolys);
    }   // end setNext

    void setPrev( int v, const ObjModel* model, const IntSet& mpolys, const IntSet& npolys)
    {
        _prev = v;
        update( model, mpolys, npolys);
    }   // end setPrev

    InnerAngle( int i, int j, int k, const ObjModel* model, const IntSet& mpolys, const IntSet& npolys)
        : _prev(i), _vidx(j), _next(k)
    {
        update( model, mpolys, npolys);
    }   // end ctor

    bool viable() const { return _viable;}
    double area() const { return _area;}
    double angle() const { return _angle;}

private:
    void update( const ObjModel* model, const IntSet& mpolys, const IntSet& npolys)
    {
        _area = calcArea( model, _prev, _vidx, _next);
        _viable = model->nspolys( _prev, _next) < 2;
        const cv::Vec3f eij = model->vtx(_prev) - model->vtx(_vidx);
        const cv::Vec3f ekj = model->vtx(_next) - model->vtx(_vidx);
        const cv::Vec3f vnrm = calcVertexNorm( model, mpolys, npolys, _prev, _vidx);  // Not normalised!
        _angle = ekj.cross(eij).dot( vnrm);
    }   // end update

    bool _viable;
    double _area;
    double _angle;
};  // end struct


// Specify the conditions that make ia1 better than ia0
bool InnerAngleComparator::operator()( const InnerAngle* ia0, const InnerAngle* ia1) const
{
    if ( ia0->viable() && ia1->viable())
    {   // Both viable so choose first based on area, then on angle.
        if ( ia0->area() > ia1->area())
            return true;
        else if ( ia0->angle() < ia1->angle())
            return true;
    }   // end if
    else if ( ia1->viable())
        return true;
    return false;
}   // end operator()


// vs is the vertex that connects to both va and vb.
// Choose how to order va and vb when setting the edge to remain consistent
// with normal of polygons adjacent to edge vs-->va and vs-->vb.
int setVertexOrderedFace( ObjModel::Ptr model, int va, int vs, int vb)
{
    // If the face already exists, return -1.
    if ( model->face( va, vs, vb) >= 0)
        return -1;

    // Default order for setting the new face is vs,va,vb.
    // Discover if the order should be vs,vb,va (swap va and vb) by looking at the vertex
    // ordering of the existing face with edge vs-->va. If va comes directly after vs in
    // the ordering of the vertices on that face, we must swap va and vb.
    int fia = *model->spolys( vs, va).begin();
    const ObjPoly& fa = model->face(fia);
    if (( fa[0] == vs && fa[1] == va) || ( fa[1] == vs && fa[2] == va) || (fa[2] == vs && fa[0] == va))
        std::swap( va, vb);
    return model->addFace( vs, va, vb); // Set the face and return its ID
}   // end setVertexOrderedFace


// Parse a list of edges applying a function iteratively to each successive vertex triplet.
struct EdgePairParser
{
    explicit EdgePairParser( const std::list<int>& lst) : _lst(lst) {}

    const std::list<int>& list() const { return _lst;}

    void parse()
    {
        std::list<int>::const_iterator last = --_lst.end();
        for ( std::list<int>::const_iterator i = _lst.begin(); i != _lst.end(); ++i)
        {
            std::list<int>::const_iterator j = i;
            if ( i == last)
                j = _lst.begin();
            else
                j++;

            std::list<int>::const_iterator k = j;
            if ( j == last)
               k = _lst.begin();
            else
                k++;

            parseEdge( *i, *j, *k);
        }   // end for
    }   // end parse

protected:
    virtual void parseEdge( int, int, int) = 0;

private:
    const std::list<int>& _lst;
};  // end struct


struct InnerAngleQueueAdder : public EdgePairParser
{
    InnerAngleQueueAdder( const std::list<int>& lst, const ObjModel* model, const IntSet& mpolys, const IntSet& npolys)
        : EdgePairParser( lst), _model(model), _mpolys(mpolys), _npolys(npolys) {}

    ~InnerAngleQueueAdder()
    {
        while ( !_queue.empty())
            erase( pop());
    }   // end dtor

    InnerAngle* pop()
    {
        InnerAngle* ia = _queue.top();
        _queue.pop();
        return ia;
    }   // end pop

    InnerAngle* get( int vj) const { return _iangles.at(vj);}

    bool empty() const { return _queue.empty();}

    void erase( InnerAngle* ia)
    {
        _iangles.erase( ia->_vidx); 
        delete ia;
    }   // end erase

    void update( InnerAngle* ia) { _queue.update( ia->_qhandle);}

protected:
    void parseEdge( int vi, int vj, int vk) override
    {
        InnerAngle* ia = new InnerAngle( vi, vj, vk, _model, _mpolys, _npolys);
        ia->_qhandle = _queue.push(ia); // O(logN)
        _iangles[vj] = ia;
        //std::cerr << "+ InnerAngle at " << vi << " --> " << vj << " --> " << vk << " with area = " << ia->area()
        //          << ", viable = " << std::boolalpha << ia->viable() << std::endl;
    }   // end parseEdge

private:
    const ObjModel* _model;
    const IntSet& _mpolys;
    const IntSet& _npolys;
    std::unordered_map<int, InnerAngle*> _iangles;
    InnerAngleQueue _queue;
};  // end struct



struct HoleFiller
{
    ObjModel::Ptr _model;
    const IntSet &_mpolys;
    IntSet &_npolys;
    InnerAngleQueueAdder _qadder;

    HoleFiller( ObjModel::Ptr model, const IntSet& mpolys, IntSet& npolys, const std::list<int>& blist)
        : _model(model), _mpolys(mpolys), _npolys(npolys), _qadder( blist, model.get(), mpolys, npolys)
    {
        _qadder.parse();
    }   // end ctor

    void fillHole()
    {
        const ObjModel* cmodel = _model.get();

        while ( !_qadder.empty())
        {
            InnerAngle* ia = _qadder.pop(); // Get the next inner angle triangle to fill

            //std::cerr << "- InnerAngle " << ia->_prev << " --> " << ia->_vidx << " --> " << ia->_next << " with area = " << ia->area()
            //          << ", viable = " << std::boolalpha << ia->viable() << std::endl;

            if ( ia->_prev != ia->_next)    // On final triangle, prev and next vertices will be set the same.
            {
                const int nfid = setVertexOrderedFace( _model, ia->_prev, ia->_vidx, ia->_next);
                if ( nfid >= 0)
                    _npolys.insert( nfid);

                // Update prev and next vertex adjacent vertex refs
                InnerAngle* ip = _qadder.get( ia->_prev);
                InnerAngle* in = _qadder.get( ia->_next);
                in->setPrev( ia->_prev, cmodel, _mpolys, _npolys);
                ip->setNext( ia->_next, cmodel, _mpolys, _npolys);
                _qadder.update(ip); // O(logN)
                _qadder.update(in); // O(logN)
            }   // end if
            _qadder.erase( ia);
        }   // end while
    }   // end fillHole
};  // end class


}   // end namespace


ObjModelHoleFiller::ObjModelHoleFiller( ObjModel::Ptr m) : _model(m) {}


int ObjModelHoleFiller::fillHole( const std::list<int>& blist, const IntSet& mpolys)
{
    _npolys.clear();
    assert( checkConnected( _model, blist));
    HoleFiller filler( _model, mpolys, _npolys, blist);
    filler.fillHole();
    return static_cast<int>(_npolys.size());
}   // end fillHole
