/**
 * Pure virtual parent of all objects that want to create
 * data structures from structured point cloud data.
 *
 * Richard Palmer
 * September 2012
 */

#pragma once
#ifndef RFEATURES_POINT_DATA_BUILDER_H
#define RFEATURES_POINT_DATA_BUILDER_H

#include <cstdlib>
#include <boost/shared_ptr.hpp>
typedef unsigned char byte;


namespace RFeatures
{

class PointDataBuilder
{
public:
    typedef boost::shared_ptr<PointDataBuilder> Ptr;

    virtual void setPointPos( int row, int col, double x, double y, double z){}
    virtual void setPointRange( int row, int col, double rng){}
    virtual void setPointCol( int row, int col, byte r, byte g, byte b){}

    virtual void reset( int width, int height) = 0;
};  // end class


}   // end RFeatures

#endif
