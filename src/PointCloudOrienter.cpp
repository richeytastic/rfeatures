#include "PointCloudOrienter.h"
using RFeatures::PointCloudOrienter;
using RFeatures::PointCloudOrienter2;
using RFeatures::PointXYZRGB;


void flipToLeft( const cv::Vec3f& u, cv::Vec3f& v)
{
    v[0] = u[2];
    v[1] = u[1];
    v[2] = -u[0];
}   // end flipToLeft

void flipToRear( const cv::Vec3f& u, cv::Vec3f& v)
{
    v[0] = -u[0];
    v[1] = u[1];
    v[2] = -u[2];
}   // end flipToRear

void flipToRight( const cv::Vec3f& u, cv::Vec3f& v)
{
    v[0] = -u[2];
    v[1] = u[1];
    v[2] = u[0];
}   // end flipToRight



cv::Mat_<cv::Vec3f> copyTo( const cv::Mat_<cv::Vec3f> pc, void (*flipFn)( const cv::Vec3f&, cv::Vec3f&))
{
    cv::Mat_<cv::Vec3f> pc2( pc.size());
    const int rows = pc.rows;
    const int cols = pc.cols;
    for ( int i = 0; i < rows; ++i)
        for ( int j = 0; j < cols; ++j)
            flipFn( pc.at<cv::Vec3f>(i,j), pc2.at<cv::Vec3f>(i,j));
    return pc2;
}   // end copyTo



PointCloud::Ptr copyTo( const PointCloud::Ptr pc, void (*flipFn)( const cv::Vec3f&, cv::Vec3f&))
{
    const int rows = pc->rows();
    const int cols = pc->cols();
    PointCloud::Ptr pc2 = PointCloud::create( cols, rows);

    float x, y, z;
    byte r, g, b;

    for ( int i = 0; i < rows; ++i)
    {
        for ( int j = 0; j < cols; ++j)
        {
            const PointXYZRGB& p1 = pc->at(i,j);
            PointCloud::unpackPoint( p1, x, y, z, r, g, b);

            cv::Vec3f u( x, y, z);
            cv::Vec3f v;
            flipFn( u, v);

            PointXYZRGB& p2 = pc2->at(i,j);
            PointCloud::packPoint( p2, v[0], v[1], v[2], r, g, b);
        }   // end for
    }   // end for

    return pc2;
}   // end copyTo



// PointCloudOrienter
PointCloudOrienter::PointCloudOrienter( const cv::Mat_<cv::Vec3f>& pc) : _pc(pc)
{ }   // end ctor

cv::Mat_<cv::Vec3f> PointCloudOrienter::copyToLeft() const
{
    return copyTo( _pc, flipToLeft);
}   // end copyToLeft

cv::Mat_<cv::Vec3f> PointCloudOrienter::copyToRear() const
{
    return copyTo( _pc, flipToRear);
}   // end copyToRear

cv::Mat_<cv::Vec3f> PointCloudOrienter::copyToRight() const
{
    return copyTo( _pc, flipToRight);
}   // end copyToRight



// PointCloudOrienter2
PointCloudOrienter2::PointCloudOrienter2( const PointCloud::Ptr pc) : _pc(pc)
{ }   // end ctor

PointCloud::Ptr PointCloudOrienter2::copyToLeft() const
{
    return copyTo( _pc, flipToLeft);
}   // end copyToLeft

PointCloud::Ptr PointCloudOrienter2::copyToRear() const
{
    return copyTo( _pc, flipToRear);
}   // end copyToRear

PointCloud::Ptr PointCloudOrienter2::copyToRight() const
{
    return copyTo( _pc, flipToRight);
}   // end copyToRight


