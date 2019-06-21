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

#include <ObjModelKDTree.h>
#include <FeatureUtils.h>
#include <kdtree.h>     // Andreas Geiger (from libICP)
using RFeatures::ObjModel;
using RFeatures::ObjModelKDTree;


class ObjModelKDTree::Impl
{
public:
    explicit Impl( const ObjModel& model)
    {
        // NB don't assume that the model vertices are in sequential order.
        const IntSet& vids = model.vtxIds();
        const int n = int(vids.size());
        _kddata.resize( boost::extents[n][3]);
        assert( _kddata.size() == vids.size());

        int i = 0;
        for ( int vid : vids)
        {
            const cv::Vec3f& v = model.uvtx(vid);   // Use the raw (untransformed) vertex
            _kddata[i][0] = v[0];
            _kddata[i][1] = v[1];
            _kddata[i][2] = v[2];
            _vvmap[i] = vid;
            _rvvmap[vid] = i++;
        }   // end for

        _kdtree = new kdtree::KDTree( _kddata);
    }   // end ctor


    ~Impl() { delete _kdtree;}


    size_t size() const { return _kddata.size();}


    int find( float x, float y, float z, float* sqdis=nullptr) const
    {
        std::vector<float> query({x,y,z});
        kdtree::KDTreeResultVector result;
        _kdtree->n_nearest( query, 1, result);
        if ( sqdis)
            *sqdis = result[0].dis;
        return _vvmap.at(result[0].idx);
    }   // end find


    int findn( float x, float y, float z, std::vector<int>& nearv, std::vector<float>* sqdis=nullptr) const
    {
        int n = (int)nearv.size();
        if ( n == 0)
            return -1;

        if ( sqdis && int(sqdis->size()) != n)
            return -1;

        std::vector<float> query({x,y,z});
        kdtree::KDTreeResultVector result;
        _kdtree->n_nearest( query, n, result);
        n = (int)result.size(); // The actual number of results

        if ( sqdis)
        {
            for ( int i = 0; i < n; ++i)
                sqdis->at(i) = result[i].dis;
        }   // end if

        for ( int i = 0; i < n; ++i)
            nearv[i] = _vvmap.at(result[i].idx);

        return n;
    }   // end findn


    int find( double x, double y, double z, float* sqdis=nullptr) const
    {
        return find( float(x), float(y), float(z), sqdis);
    }   // end find


    int findn( double x, double y, double z, std::vector<int>& nearv, std::vector<float>* sqdis=nullptr) const
    {
        return findn( float(x), float(y), float(z), nearv, sqdis);
    }   // end findn


    cv::Vec3f pos( int vidx) const
    {
        const auto& v = _kddata[_rvvmap.at(vidx)];
        return cv::Vec3f( v[0], v[1], v[2]);
    }   // end pos


private:
    kdtree::KDTreeArray _kddata;
    kdtree::KDTree *_kdtree;
    std::unordered_map<int,int> _vvmap, _rvvmap;
};  // end class


// public
ObjModelKDTree::Ptr ObjModelKDTree::create( const ObjModel& model)
{
    return Ptr( new ObjModelKDTree( model), [](ObjModelKDTree* d){delete d;});
}   // end create


// private
ObjModelKDTree::ObjModelKDTree( const ObjModel& model)
    : _impl( new Impl(model)), _tmat( model.transformMatrix()), _imat( _tmat.inv())
{
}   // end ctor


// private
ObjModelKDTree::~ObjModelKDTree() { delete _impl;}


size_t ObjModelKDTree::numVtxs() const { return _impl->size();}


int ObjModelKDTree::find( const cv::Vec3d& p, float* sqdis) const
{
    const cv::Vec3d q = transform( _imat, p);
    return _impl->find( q[0], q[1], q[2], sqdis);
}   // end find


int ObjModelKDTree::find( const cv::Vec3f& p, float* sqdis) const
{
    const cv::Vec3f q = transform( _imat, p);
    return _impl->find( q[0], q[1], q[2], sqdis);
}   // end find


int ObjModelKDTree::findn( const cv::Vec3d& p, std::vector<int>& nv, std::vector<float>* sqdis) const
{
    const cv::Vec3d q = transform( _imat, p);
    return _impl->findn( q[0], q[1], q[2], nv, sqdis);
}   // end findn


int ObjModelKDTree::findn( const cv::Vec3f& p, std::vector<int>& nv, std::vector<float>* sqdis) const
{
    const cv::Vec3f q = transform( _imat, p);
    return _impl->findn( q[0], q[1], q[2], nv, sqdis);
}   // end findn


cv::Vec3f ObjModelKDTree::pos( int vidx) const
{
    return transform( _tmat, _impl->pos(vidx));
}   // end pos


void ObjModelKDTree::setTransformMatrix( const cv::Matx44d& tmat)
{
    _tmat = tmat;
    _imat = tmat.inv();
}   // end setTransformMatrix
