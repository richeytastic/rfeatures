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

/**
 * Abstract factory for reading in structured point data in any format and building
 * abstract data types.
 *
 * Richard Palmer
 * August 2012
 */

#ifndef RFEATURES_POINT_DATA_READER
#define RFEATURES_POINT_DATA_READER

#ifdef _WIN32
#pragma warning( disable : 4251)
#endif

#include <cstdlib>
#include <iostream>
#include <list>

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
    virtual void read( std::istream &is) = 0;

    // Child classes must first provide the size of the structured point cloud
    // (as read in from the stream) before point data can be added to the member objects.
    virtual void getSize( std::istream &is, size_t &width, size_t &height) = 0;

    // Child classes must implement this function for the reading in of each point and
    // the setting of the provided position (x,y,z) and colour (r,g,b) parameters.
    // It is not strictly necessary for the implementing class to read in each point
    // from the provided stream prior to calling this function; indeed it is usually
    // cheaper to read in the data as a whole in a single system call (especially if
    // reading from disk or over the network). This function is called in the order of
    // top row to bottom row and from leftmost column to rightmost column.
    virtual void getPoint( std::istream &is, size_t row, size_t col,
                    double &x, double &y, double &z, double &rng, byte &r, byte &g, byte &b) = 0;

    // Called once all builder objects have been parsed on the read.
    virtual void finishedRead(){}

    PointDataReader();  // No non-derived class construction

private:
    std::list<PointDataBuilder::Ptr> builders;   // Builders that will use data read in by this object.

    void readStream( std::istream &is);    // Called by operator>> (calls virtual functions)
    friend std::istream &operator>>( std::istream &is, PointDataReader&);

    // No copy construction
    PointDataReader( const PointDataReader&);
    void operator=( const PointDataReader&);
};  // end class PointDataReader


std::istream &operator>>( std::istream &is, PointDataReader&);


}   // end namespace RFeatures

#endif
