/**
 * Defines constructs to use cv::Vec3f and cv::Vec2f types as keys for hash tables.
 * Richard Palmer
 * September 2014
 */

#pragma once
#ifndef RFEATURES_VECTOR_FLOAT_KEY_HASHING_H
#define RFEATURES_VECTOR_FLOAT_KEY_HASHING_H

#include <cmath>
#include <opencv2/opencv.hpp>
namespace cv
{
    typedef Vec<int,6> Vec6i;
}   // end namespace

#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>
#include <boost/functional/hash.hpp>

#include "rFeatures_Export.h"

namespace RFeatures
{

// A point in space is uniquely described by its position and its normal.
// Take two cv::Vec3fs and convert to a single cv::Vec6i (for hashing purposes).
// Default precision is six decimal places.
cv::Vec6i concatToInt( const cv::Vec3f& u, const cv::Vec3f& v, int pw=6);
cv::Vec3i toInt( const cv::Vec3f& u, int pw=6);
cv::Vec2i toInt( const cv::Vec2f& u, int pw=6);

struct rFeatures_EXPORT HashVec6i : std::unary_function<cv::Vec6i, size_t>
{
    size_t operator()( const cv::Vec6i& u) const;
};  // end struct

struct rFeatures_EXPORT HashVec3i : std::unary_function<cv::Vec3i, size_t>
{
    size_t operator()( const cv::Vec3i& u) const;
};  // end struct

struct rFeatures_EXPORT HashVec2i : std::unary_function<cv::Vec2i, size_t>
{
    size_t operator()( const cv::Vec2i& u) const;
};  // end struct

/*
struct EquateVec3i : std::binary_function<cv::Vec3i, cv::Vec3i, bool>
{
    bool operator()( const cv::Vec3i& u, const cv::Vec3i& v) const
    {
        return (x[0] == y[0]) && (x[1] == y[1]) && (x[2] == y[2]);
    }   // end operator()
};  // end struct
struct EquateVec2i : std::binary_function<cv::Vec2i, cv::Vec2i, bool>
{
    bool operator()( const cv::Vec2i& u, const cv::Vec2i& v) const
    {
        return (x[0] == y[0]) && (x[1] == y[1]);
    }   // end operator()
};  // end struct
*/

typedef boost::unordered_map<cv::Vec6i, int, HashVec6i> Vec6iToIntMap;
typedef boost::unordered_map<cv::Vec3i, int, HashVec3i> Vec3iToIntMap;
typedef boost::unordered_map<cv::Vec2i, int, HashVec2i> Vec2iToIntMap;

}   // end namespace

#endif
