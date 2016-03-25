#include "PointDataWriter.h"
using RFeatures::PointDataWriter;


ostream &RFeatures::operator<<( ostream &os, const PointDataWriter &pdr)
{
    pdr.write(os);
    return os;
}   // end operator<<
