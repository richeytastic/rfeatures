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
