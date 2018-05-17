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

#include <ObjModelKDTree.h>
#include <kdtree.h>     // Andreas Geiger (from libICP)
using RFeatures::ObjModel;
using RFeatures::ObjModelKDTree;

class ObjModelKDTree::Deleter
{ public:
    void operator()( ObjModelKDTree *t) { delete t;}
};  // end class


// public
ObjModelKDTree::Ptr ObjModelKDTree::create( const ObjModel::Ptr model)
{
    return Ptr( new ObjModelKDTree( model), Deleter());
}   // end create


// public
ObjModelKDTree::Ptr ObjModelKDTree::create( const ObjModel::Ptr model, const IntSet& vidxs)
{
    return Ptr( new ObjModelKDTree( model, vidxs), Deleter());
}   // end create



class ObjModelKDTree::Impl
{
public:
    void init( const ObjModel::Ptr model, const IntSet& vidxs)
    {
        _kddata = new kdtree::KDTreeArray;
        const int n = (int)vidxs.size();
        _kddata->resize( boost::extents[n][3]);
        _vmap = new std::vector<int>(n);

        int i = 0;
        for ( int vidx : vidxs)
        {
            const cv::Vec3f& v = model->vtx(vidx);
            (*_kddata)[i][0] = v[0];
            (*_kddata)[i][1] = v[1];
            (*_kddata)[i][2] = v[2];
            _vmap->at(i++) = vidx;
        }   // end foreach
        _kdtree = new kdtree::KDTree( *_kddata);
    }   // end init

    Impl( const ObjModel::Ptr model)
    {
        init( model, model->getVertexIds());
    }   // end ctor

    Impl( const ObjModel::Ptr model, const IntSet& vidxs)
    {
        init( model, vidxs);
    }   // end ctor

    ~Impl()
    {
        delete _kdtree;
        delete _kddata;
        delete _vmap;
    }   // end ctor


    int find( const cv::Vec3f& p, float* sqdis=NULL) const
    {
        std::vector<float> query(3);
        query[0] = p[0];
        query[1] = p[1];
        query[2] = p[2];

        kdtree::KDTreeResultVector result;
        _kdtree->n_nearest( query, 1, result);
        if ( sqdis)
            *sqdis = result[0].dis;
        return _vmap->at(result[0].idx);
    }   // end find


    int findn( const cv::Vec3f& p, std::vector<int>& nearv, std::vector<float>* sqdis=NULL) const
    {
        std::vector<float> query(3);
        query[0] = p[0];
        query[1] = p[1];
        query[2] = p[2];

        int n = (int)nearv.size();
        if ( n == 0)
            return -1;

        if ( sqdis && int(sqdis->size()) != n)
            return -1;

        kdtree::KDTreeResultVector result;
        _kdtree->n_nearest( query, n, result);
        n = (int)result.size(); // The actual number of results

        if ( sqdis)
        {
            for ( int i = 0; i < n; ++i)
                sqdis->at(i) = result[i].dis;
        }   // end if

        for ( int i = 0; i < n; ++i)
            nearv[i] = _vmap->at(result[i].idx);

        return n;
    }   // end findn


    int find( const cv::Vec3d& p, float* sqdis=NULL) const
    {
        return find( cv::Vec3f( float(p[0]), float(p[1]), float(p[2])), sqdis);
    }   // end find


    int findn( const cv::Vec3d& p, std::vector<int>& nearv, std::vector<float>* sqdis=NULL) const
    {
        return findn( cv::Vec3f( float(p[0]), float(p[1]), float(p[2])), nearv, sqdis);
    }   // end findn

private:
    std::vector<int>* _vmap; // Vertex map for _kddata
    kdtree::KDTree *_kdtree;
    kdtree::KDTreeArray *_kddata;
};  // end class


// private
ObjModelKDTree::ObjModelKDTree( const ObjModel::Ptr model) : _model(model), _impl( new Impl(model)) {}
ObjModelKDTree::ObjModelKDTree( const ObjModel::Ptr model, const IntSet& vidxs) : _model(model), _impl( new Impl(model, vidxs)) {}
ObjModelKDTree::~ObjModelKDTree() { delete _impl;}


// public
int ObjModelKDTree::find( const cv::Vec3d& p, float* sqdis) const { return _impl->find( p, sqdis);}
int ObjModelKDTree::find( const cv::Vec3f& p, float* sqdis) const { return _impl->find( p, sqdis);}
int ObjModelKDTree::findn( const cv::Vec3d& p, std::vector<int>& nv, std::vector<float>* sqdis) const { return _impl->findn( p, nv, sqdis);}
int ObjModelKDTree::findn( const cv::Vec3f& p, std::vector<int>& nv, std::vector<float>* sqdis) const { return _impl->findn( p, nv, sqdis);}

