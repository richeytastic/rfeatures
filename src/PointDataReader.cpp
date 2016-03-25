#include "PointDataReader.h"
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



void PointDataReader::readStream( istream &is)
{
    this->read( is);  // Allow child to read in as much data as necessary

    size_t width, height;
    getSize( is, width, height);
    BOOST_FOREACH( PointDataBuilder::Ptr &p, builders)
        p->reset( width, height);

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
                p->setPointPos( row, col, x, y, z);
                p->setPointCol( row, col, r, g, b);
                p->setPointRange( row, col, rng);
            }   // end foreach
        }   // end for
    }   // end for

    finishedRead();
}   // end readStream



istream &RFeatures::operator>>( istream &is, PointDataReader &pdr)
{
    pdr.readStream(is);
    return is;
}   // end operator>>
