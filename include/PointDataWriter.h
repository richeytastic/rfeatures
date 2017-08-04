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
