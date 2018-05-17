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

#ifndef RFEATURES_OBJ_MODEL_KD_TREE_H
#define RFEATURES_OBJ_MODEL_KD_TREE_H

#include "ObjModel.h"

namespace RFeatures {

class rFeatures_EXPORT ObjModelKDTree
{
public:
    typedef std::shared_ptr<ObjModelKDTree> Ptr;

    // Don't modify the given model while this data structure in use.
    static Ptr create( const ObjModel*);
    static Ptr create( const ObjModel*, const IntSet& vidxs);   // Create from just the given vertex IDs

    const ObjModel* model() const { return _model;}

    // Find the closest vertex ID on the model that are closest to p.
    // If not NULL, set sqdis on return to be the squared distance to the vertex.
    int find( const cv::Vec3f& p, float *sqdis=NULL) const;
    int find( const cv::Vec3d& p, float *sqdis=NULL) const;

    // Find the closest n vertex IDs on the model that are closest to p. n is specified by
    // preallocating nvidxs to the desired size. If sqdis is given, it must also have
    // the same size as nvidxs. Returns the actual number of points found which may be
    // less than n.
    int findn( const cv::Vec3f& p, std::vector<int>& nvidxs, std::vector<float>* sqdis=NULL) const;
    int findn( const cv::Vec3d& p, std::vector<int>& nvidxs, std::vector<float>* sqdis=NULL) const;

private:
    const ObjModel* _model;
    class Impl; // pimple idiom (who doesn't love saying that...)
    Impl *_impl;

    explicit ObjModelKDTree( const ObjModel*);
    ObjModelKDTree( const ObjModel*, const IntSet&);
    ~ObjModelKDTree();
    ObjModelKDTree( const ObjModelKDTree&); // No copy
    ObjModelKDTree& operator=( const ObjModelKDTree&);  // No copy
};  // end class

}   // end namespace

#endif
