#include "PointCloudWriter.h"
using RFeatures::PointCloudWriter;


PointCloudWriter::PointCloudWriter( const PointCloud::Ptr &pc)
    : pcloud_(pc)
{}   // end ctor


PointCloudWriter::~PointCloudWriter()
{}   // end dtor
