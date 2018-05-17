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

#include <PointDataReader.h>
#include <boost/foreach.hpp>
using RFeatures::PointDataReader;


PointDataReader::PointDataReader()
{
}   // end ctor


PointDataReader::~PointDataReader()
{
    builders.clear();
}   // end dtor



size_t PointDataReader::addDataBuilder( const PointDataBuilder::Ptr &pcob)
{
    builders.push_back(pcob);
    return builders.size();
}   // end addDataBuilder



void PointDataReader::readStream( std::istream &is)
{
    this->read( is);  // Allow child to read in as much data as necessary

    size_t width, height;
    getSize( is, width, height);
    BOOST_FOREACH( PointDataBuilder::Ptr &p, builders)
        p->reset( (int)width, (int)height);

    double x, y, z;
    double rng;
    byte r, g, b;
    for ( size_t row = 0; row < height; ++row)
    {
        for ( size_t col = 0; col < width; ++col)
        {
            getPoint( is, row, col, x, y, z, rng, r, g, b);
            BOOST_FOREACH( PointDataBuilder::Ptr &p, builders)
            {
                p->setPointPos( (int)row, (int)col, x, y, z);
                p->setPointCol( (int)row, (int)col, r, g, b);
                p->setPointRange( (int)row, (int)col, rng);
            }   // end foreach
        }   // end for
    }   // end for

    finishedRead();
}   // end readStream



std::istream &RFeatures::operator>>( std::istream &is, PointDataReader &pdr)
{
    pdr.readStream(is);
    return is;
}   // end operator>>
