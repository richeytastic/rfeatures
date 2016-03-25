/**
 * Abstract parent class for View writer types.
 *
 * Richard Palmer
 * September 2012
 */

#pragma once
#ifndef RFEATURES_POINT_CLOUD_WRITER
#define RFEATURES_POINT_CLOUD_WRITER

#include "PointDataWriter.h"
using RFeatures::PointDataWriter;
#include "PointCloud.h"
using RFeatures::PointCloud;


namespace RFeatures
{

class rFeatures_EXPORT PointCloudWriter : public PointDataWriter
{
public:
    virtual ~PointCloudWriter();

protected:
    virtual void write( ostream &os) const = 0;   // Implemented in device specific child classes

    explicit PointCloudWriter( const PointCloud::Ptr&); // No non-derived class construction
    const PointCloud::Ptr pcloud_;   // Object being written out
};  // end class PointCloudWriter


}   // end namespace

#endif
