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

#ifndef RFEATURES_VECTOR_FLOAT_KEY_HASHING_H
#define RFEATURES_VECTOR_FLOAT_KEY_HASHING_H

#include "rFeatures_Export.h"
#include <opencv2/opencv.hpp>
#include <unordered_map>
#include <unordered_set>
#include <iostream>
#include <cassert>
#include <cmath>

namespace RFeatures {

template <typename T, int M>
class Key
{
public:
    Key( double v[M], int pw=6);
    Key( float v[M], int pw=6);
    const T& operator[]( int i) const { return _ielems[i];}
    bool operator==( const Key<T,M>&) const;
private:
    T _ielems[M];
};  // end class

template <typename T, int M>
std::ostream& operator<<( std::ostream&, const Key<T,M>&);

#include "template/VectorFloatKeyHashing_template.h"

typedef Key<long,6> Key6L;
typedef Key<long,3> Key3L;
typedef Key<long,2> Key2L;

rFeatures_EXPORT double roundDP( double v, int pw=6); // Round v to p decimal places and return

// Default precision is six decimal places. WATCH OUT FOR UNDERFLOW!
rFeatures_EXPORT Key6L concatToKey( const cv::Vec3f& u, const cv::Vec3f& v, int pw=6);
rFeatures_EXPORT Key3L toKey( double x, double y, double z, int pw=6);
rFeatures_EXPORT Key3L toKey( const cv::Vec3f& u, int pw=6);
rFeatures_EXPORT Key2L toKey( double x, double y, int pw=6);
rFeatures_EXPORT Key2L toKey( const cv::Vec2f& u, int pw=6);

struct rFeatures_EXPORT HashKey6L : std::unary_function<Key6L, size_t> { size_t operator()( const Key6L&) const;};
struct rFeatures_EXPORT HashKey3L : std::unary_function<Key3L, size_t> { size_t operator()( const Key3L&) const;};
struct rFeatures_EXPORT HashKey2L : std::unary_function<Key2L, size_t> { size_t operator()( const Key2L&) const;};

typedef std::unordered_map<Key6L, int, HashKey6L> Key6LToIntMap;
typedef std::unordered_map<Key3L, int, HashKey3L> Key3LToIntMap;
typedef std::unordered_map<Key2L, int, HashKey2L> Key2LToIntMap;

typedef std::unordered_set<Key6L, HashKey6L> Key6LSet;
typedef std::unordered_set<Key3L, HashKey3L> Key3LSet;
typedef std::unordered_set<Key2L, HashKey2L> Key2LSet;

}   // end namespace

#endif
