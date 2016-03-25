#include "RangeImager.h"
using namespace RFeatures;
#include <cmath>


const float RangeImager::NO_DISTANCE_VAL( -1);


RangeImager::RangeImager( const PointCloud::Ptr pc, const Vector3d &fv, float fl) throw (PointCloudException, InvalidVectorException)
{
    if ( !pc->isOrganised())
        throw PointCloudException( "Cannot produce range image from unorganised point cloud!");
    if ( fv.norm() == 0.0)
        throw InvalidVectorException( "Zero vector cannot be used as focal vector!");
    pcloud = pc;
    focLen = fl;
    f = fv.normalized();
    reset();
}   // end setPointCloud



cv::Mat RangeImager::getRangeData() const
{
    return rngData;
}   // end getRangeData



cv::Mat RangeImager::getRangeImage() const
{
    return greyImg;
}   // end getRangeImage



PointCloud::Ptr RangeImager::createPointCloud()
{
    if ( isQuad)
        reset();

    PointCloud::Ptr pc = PointCloud::create( pcloud->getWidth(), pcloud->getHeight());
    if ( focLen <= 0.0)
        return pc;

    float xcentre = (float)(rngData.cols - 1)/2;
    float ycentre = (float)(rngData.rows - 1)/2;
    float x, y, z;
    byte r, g, b;
    for ( int yi = 0; yi < rngData.rows; ++yi)
    {
        for ( int xi = 0; xi < rngData.cols; ++xi)
        {
            x = 0;
            y = 0;
            z = 0;
            r = 0;
            g = 0;
            b = 0;

            if ( rngData.at<float>(yi,xi) != NO_DISTANCE_VAL)
            {
                pcloud->from( yi, xi, x, y, z, r, g, b);    // Get colour info (we recompute the x,y,z)
                z = rngData.at<float>(yi,xi);
                x = z * (xcentre - (float)xi) / focLen;
                y = z * (ycentre - (float)yi) / focLen;
            }   // end if

            pc->set( yi, xi, x, y, z, r, g, b);
        }   // end for
    }   // end for

    return pc;
}   // end createPointCloud



void RangeImager::reset()
{
    createRangeData();
    createRangeImage( m_maxDist);
}   // end reset



cv::Mat RangeImager::operator()() const
{
    return getRangeData();
}   // end operator()



void RangeImager::createRangeData() throw (InvalidVectorException)
{
    int xdim = pcloud->getWidth();
    int ydim = pcloud->getHeight();
    rngData = cv::Mat( cv::Size( xdim, ydim), CV_32FC1, cv::Scalar(0.0));

    float dist = NO_DISTANCE_VAL;
    m_maxDist = INT_MIN;
    for ( int yi = 0; yi < ydim; ++yi)
    {
        for ( int xi = 0; xi < xdim; ++xi)
        {
            const pcl::PointXYZRGB &pt = pcloud->at( yi, xi);

            // Ignore points without valid depth
            dist = NO_DISTANCE_VAL;
            if ( pt.x != 0.0 || pt.y != 0.0 || pt.z != 0.0)
            {
                Vector3d p( pt.x, pt.y, pt.z);
                dist = f.dot(p);    // Distance of p from projection plane
                if ( dist < 0.0)
                    throw InvalidVectorException( "Provided focal vector does not point into scene!");
                if ( dist > m_maxDist) m_maxDist = dist;
            }   // end if

            rngData.at<float>( yi, xi) = dist;
        }   // end for
    }   // end for
}   // end createRangeData



cv::Mat RangeImager::createGenericRangeImage( float scale, uchar (*getRangeVal)( float, float) )
{
    cv::Mat gImg( rngData.size(), CV_8UC1, cv::Scalar(0));    // Black image

    for ( int yi = 0; yi < rngData.rows; ++yi)
    {
        for ( int xi = 0; xi < rngData.cols; ++xi)
        {
            if ( rngData.at<float>(yi,xi) != NO_DISTANCE_VAL)
                gImg.at<uchar>( yi, xi) = getRangeVal( scale, rngData.at<float>(yi,xi));
        }   // end for
    }   // end for

    greyImg = gImg.clone(); // Clone for member

    return gImg;
}   // end createGenericRangeImage



float getLinearScale( float maxDist)
{
    return -255.0 / maxDist;
}   // end getLinearScale


float getQuadraticScale( float maxDist)
{
    return -255.0 / sqrt(maxDist);
}   // end getQuadraticScale


uchar getLinearRange( float scale, float r)
{
    return cv::saturate_cast<uchar>(scale * r + 255);
}   // end getLinearRange


uchar getQuadraticRange( float scale, float r)
{
    return cv::saturate_cast<uchar>(scale * sqrt(r) + 255);
}   // end getQuadraticRange



cv::Mat RangeImager::createRangeImage( float maxDist)
{
    isQuad = false;
    if ( maxDist < 1.0)
        maxDist = 1.0;
    m_maxDist = maxDist;    // Record the maximum range used for this image
    return createGenericRangeImage( getLinearScale( maxDist), getLinearRange);
}   // end createRangeImage



cv::Mat RangeImager::createQuadraticRangeImage( float maxDist)
{
    isQuad = true;
    if ( maxDist < 1.0)
        maxDist = 1.0;
    m_maxDist = maxDist;    // Record the maximum range used for this image
    return createGenericRangeImage( getQuadraticScale( maxDist), getQuadraticRange);
}   // end createQuadraticRangeImage
