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
#include <opencv2/flann.hpp>
using RFeatures::ObjModel;
using RFeatures::ObjModelKDTree;


class ObjModelKDTree::Impl
{
public:
    explicit Impl( const ObjModel& model)
    {
        // NB don't assume that the model vertices are in sequential order.
        const IntSet& vids = model.vtxIds();
        _vpts.resize(vids.size());

        int i = 0;
        for ( int vid : vids)
        {
            // Use the raw (untransformed) vertex
            _vpts[i] = model.uvtx(vid);
            _vvmap[i] = vid;
            _rvvmap[vid] = i++;
        }   // end for

        _kdtree = new cv::flann::Index( cv::Mat(_vpts).reshape(1), cv::flann::KDTreeIndexParams());
    }   // end ctor


    ~Impl() { delete _kdtree;}


    size_t size() const { return _vpts.size();}


    int find( const cv::Vec3f& q, float* sqdis=nullptr) const
    {
        std::vector<int> idx(1);
        std::vector<float> dist(1);
        _kdtree->knnSearch( q, idx, dist, 1, cv::flann::SearchParams());
        if ( sqdis)
            *sqdis = dist[0];
        return _vvmap.at(idx[0]);
    }   // find


    int findn( const cv::Vec3f& q, std::vector<int>& nearv, std::vector<float>* sqdis=nullptr) const
    {
        int n = (int)nearv.size();
        if ( n == 0)
            return -1;

        if ( sqdis && int(sqdis->size()) != n)
            return -1;

        std::vector<float> ds(n);
        if ( !sqdis)
            sqdis = &ds;

        _kdtree->knnSearch( q, nearv, *sqdis, n, cv::flann::SearchParams());
        n = (int)nearv.size(); // The actual number of results
        for ( int i = 0; i < n; ++i)
            nearv[i] = _vvmap.at(nearv[i]);

        return n;
    }   // end findn


    const cv::Vec3f& pos( int vidx) const { return _vpts[_rvvmap.at(vidx)]; }   // end pos

private:
    std::vector<cv::Vec3f> _vpts;
    cv::flann::Index *_kdtree;
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
    return _impl->find( q, sqdis);
}   // end find


int ObjModelKDTree::find( const cv::Vec3f& p, float* sqdis) const
{
    const cv::Vec3f q = transform( _imat, p);
    return _impl->find( q, sqdis);
}   // end find


int ObjModelKDTree::findn( const cv::Vec3d& p, std::vector<int>& nv, std::vector<float>* sqdis) const
{
    const cv::Vec3d q = transform( _imat, p);
    return _impl->findn( q, nv, sqdis);
}   // end findn


int ObjModelKDTree::findn( const cv::Vec3f& p, std::vector<int>& nv, std::vector<float>* sqdis) const
{
    const cv::Vec3f q = transform( _imat, p);
    return _impl->findn( q, nv, sqdis);
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
