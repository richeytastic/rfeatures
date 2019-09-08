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
#include <nanoflann.hpp>
using RFeatures::ObjModel;
using RFeatures::ObjModelKDTree;
using RFeatures::ObjModelPoints;


namespace {
typedef nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<float, ObjModelPoints<float> >, ObjModelPoints<float>, 3> MyKDTree;
}   // end namespace


class ObjModelKDTree::Impl
{
public:
    explicit Impl( const ObjModel& model) : _model(model), _pcloud(model)
    {
        _kdtree = new MyKDTree( 3, _pcloud, nanoflann::KDTreeSingleIndexAdaptorParams(10));
        _kdtree->buildIndex();
    }   // end ctor


    ~Impl() { delete _kdtree;}


    size_t size() const { return _pcloud.kdtree_get_point_count();}


    int find( const cv::Vec3f& q, float* sqdis=nullptr) const
    {
        size_t fvid;
        float sd;
        _kdtree->knnSearch( &q[0], 1, &fvid, &sd);
        if ( sqdis)
            *sqdis = sd;
        return static_cast<int>(fvid);
    }   // end find


    int findn( const cv::Vec3f& q, std::vector<int>& nearv, std::vector<float>* sqdis=nullptr) const
    {
        int n = (int)nearv.size();
        if ( n == 0)
            return -1;

        if ( sqdis && int(sqdis->size()) != n)
            return -1;

        std::vector<size_t> nidxs(n);
        std::vector<float> sds(n);

        n = _kdtree->knnSearch( &q[0], n, &nidxs[0], &sds[0]);
        nidxs.resize(n);
        nearv.resize(n);
        sds.resize(n);

        if ( sqdis)
            *sqdis = sds;   // Copy out

        for ( int i = 0; i < n; ++i)
            nearv[i] = static_cast<int>(nidxs[i]);

        return n;
    }   // end findn


    const cv::Vec3f& pos( int vidx) const { return _model.vtx(vidx);}   // end pos

private:
    const ObjModel &_model;
    const ObjModelPoints<float> _pcloud;
    MyKDTree *_kdtree;
};  // end class


// public
ObjModelKDTree::Ptr ObjModelKDTree::create( const ObjModel& model)
{
    return Ptr( new ObjModelKDTree( model), [](ObjModelKDTree* d){delete d;});
}   // end create


// private
ObjModelKDTree::ObjModelKDTree( const ObjModel& model)
    : _impl( new Impl(model)), _tmat( model.transformMatrix()), _imat( _tmat.inv()) {}


// private
ObjModelKDTree::~ObjModelKDTree() { delete _impl;}


size_t ObjModelKDTree::numVtxs() const { return _impl->size();}


int ObjModelKDTree::find( const cv::Vec3d& p, float* sqdis) const
{
    const cv::Vec3f q = transform( _imat, p);
    return _impl->find( q, sqdis);
}   // end find


int ObjModelKDTree::find( const cv::Vec3f& p, float* sqdis) const
{
    const cv::Vec3f q = transform( _imat, p);
    return _impl->find( q, sqdis);
}   // end find


int ObjModelKDTree::findn( const cv::Vec3d& p, std::vector<int>& nv, std::vector<float>* sqdis) const
{
    const cv::Vec3f q = transform( _imat, p);
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
