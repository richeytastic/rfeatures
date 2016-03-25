/**
 * Abstract factory for writing out point data in any format.
 *
 * Richard Palmer
 * September 2012
 */

#pragma once
#ifndef RFEATURES_POINT_DATA_WRITER
#define RFEATURES_POINT_DATA_WRITER

#include <cstdlib>
#include <iostream>
using std::ostream;
#include "rFeatures_Export.h"


namespace RFeatures
{

class rFeatures_EXPORT PointDataWriter
{
public:
    virtual ~PointDataWriter(){}

protected:
    // Child objects implement this function to write out data.
    virtual void write( ostream&) const = 0;
    friend ostream &operator<<( ostream&, const PointDataWriter&);

    PointDataWriter(){}  // No non-derived class construction

private:
    // No copy construction
    PointDataWriter( const PointDataWriter&);
    void operator=( const PointDataWriter&);
};  // end class


ostream &operator<<( ostream &, const PointDataWriter&);

}   // end namespace

#endif
