/**
 * Abstract factory for reading in structured point data in any format and building
 * abstract data types.
 *
 * Richard Palmer
 * August 2012
 */

#pragma once
#ifndef RFEATURES_POINT_DATA_READER
#define RFEATURES_POINT_DATA_READER

#include <cstdlib>
#include <iostream>
using std::istream;
#include <list>
using std::list;
#include <boost/foreach.hpp>

#include "PointDataBuilder.h"
using RFeatures::PointDataBuilder;
#include "rFeatures_Export.h"

typedef unsigned char byte;


namespace RFeatures
{

class rFeatures_EXPORT PointDataReader
{
public:
    virtual ~PointDataReader();

    // Adds a point data builder instance to this reader and returns the total
    // number of point data builder instances added so far. All read operations
    // will apply to all PointDataBuilder instances added using this method.
    size_t addDataBuilder( const PointDataBuilder::Ptr &pcob);

protected:
    // Child objects implement this function to read in initial or all data as needed.
    virtual void read( istream &is) = 0;

    // Child classes must first provide the size of the structured point cloud
    // (as read in from the stream) before point data can be added to the member objects.
    virtual void getSize( istream &is, size_t &width, size_t &height) = 0;

    // Child classes must implement this function for the reading in of each point and
    // the setting of the provided position (x,y,z) and colour (r,g,b) parameters.
    // It is not strictly necessary for the implementing class to read in each point
    // from the provided stream prior to calling this function; indeed it is usually
    // cheaper to read in the data as a whole in a single system call (especially if
    // reading from disk or over the network). This function is called in the order of
    // top row to bottom row and from leftmost column to rightmost column.
    virtual void getPoint( istream &is, size_t row, size_t col,
                    double &x, double &y, double &z, double &rng, byte &r, byte &g, byte &b) = 0;

    // Called once all builder objects have been parsed on the read.
    virtual void finishedRead(){}

    PointDataReader();  // No non-derived class construction

private:
    list<PointDataBuilder::Ptr> builders;   // Builders that will use data read in by this object.

    void readStream( istream &is);    // Called by operator>> (calls virtual functions)
    friend istream &operator>>( istream &is, PointDataReader&);

    // No copy construction
    PointDataReader( const PointDataReader&);
    void operator=( const PointDataReader&);
};  // end class PointDataReader


istream &operator>>( istream &is, PointDataReader&);


}   // end namespace RFeatures

#endif
