#include "PointCloudReader.h"
using RFeatures::PointCloudReader;
#include <cassert>
#include <climits>


PointCloudReader::PointCloudReader()
    : pcloud_( PointCloud::create())    // Not initially structured!
{
}   // end ctor



void PointCloudReader::getSize( istream &is, size_t &width, size_t &height)
{
    width = pcloud_->cols();
    height = pcloud_->rows();
}   // end getSize



void PointCloudReader::getPoint( istream &is, size_t row, size_t col,
        double &x, double &y, double &z, double &rng, byte &r, byte &g, byte &b)
{
    pcloud_->from( row, col, x, y, z, r, g, b);  // Set x,y,z,r,g,b
    rng = z;
}   // end getPoint



void PointCloudReader::finishedRead()
{
    return; // No-op
}   // end finishedRead



void PointCloudReader::createPointCloud( size_t rows, size_t cols)
{
    pcloud_ = PointCloud::create( cols, rows);
}   // end createPointCloud



void PointCloudReader::setPoint( size_t row, size_t col, double x, double y, double z, byte r, byte g, byte b)
{
    pcloud_->set( row, col, x, y, z, r, g, b);
}   // end setPoint


/*
istream &RFeatures::operator>>( istream &is, PointCloudReader::Ptr &vr)
{
    PointDataReader::Ptr pdr( boost::static_pointer_cast<PointDataReader>(vr));
    return is >> pdr;
}   // end operator>>
*/
