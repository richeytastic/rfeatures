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

#ifndef RFEATURES_DESCRIPTOR_STATISTICS_H
#define RFEATURES_DESCRIPTOR_STATISTICS_H

#ifdef _WIN32
#pragma warning( disable : 4290)
#endif

#include "RFeatures.h"
using std::ostream;
using std::istream;
#include "FeatureExceptions.h"
using RFeatures::DescriptorLengthException;


namespace RFeatures
{

class rFeatures_EXPORT DescriptorStatistics
{
public:
    // Add a new descriptor - returns number added so far. All added
    // descriptors must be of the same length or exception is thrown.
    int add( const cv::Mat& descriptor) throw (DescriptorLengthException);

private:
    cv::Mat _vecs;
    friend ostream& operator<<( ostream& os, const DescriptorStatistics&);
};  // end class


// Prints stats to the given output stream.
ostream& operator<<( ostream& os, const DescriptorStatistics&);

// Read in a descriptor formatted as space separated
// numerical text values on a single line.
istream& operator>>( istream& is, DescriptorStatistics&);

// Synonymous with ds.add(descriptor)
DescriptorStatistics& operator<<( DescriptorStatistics& ds, const cv::Mat& descriptor);

}   // end namespace

#endif
