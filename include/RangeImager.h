/**
 * Create a grey scale range image from an organised point cloud.
 */

#ifndef RFEATURES_RANGE_IMAGER_H
#define RFEATURES_RANGE_IMAGER_H

#include <exception>
#include <string>
#include "PointCloud.h"
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
using Eigen::Vector3d;
using Eigen::MatrixXd;


namespace RFeatures
{

class InvalidVectorException : public std::exception
{
public:
    InvalidVectorException( const std::string &err) : m_err(err){}
    virtual ~InvalidVectorException() throw(){}
    virtual const char* what() const throw(){ return m_err.c_str();}
    virtual std::string error() const throw(){ return m_err;}
    virtual std::string errStr() const throw(){ return m_err;}
private:
    std::string m_err;
}; // end class InvalidVectorException



class RangeImageException : public std::exception
{
public:
    RangeImageException( const cv::Mat &rimg, const std::string &err) : m_err(err), img(rimg){}
    virtual ~RangeImageException() throw(){}
    virtual const char* what() const throw(){ return m_err.c_str();}
    virtual std::string error() const throw(){ return m_err;}
    virtual std::string errStr() const throw(){ return m_err;}
    virtual cv::Mat errImg() const throw(){ return img;}
private:
    std::string m_err;
    const cv::Mat &img;
};  // end class RangeImageException



class RangeImager
{
public:
    // Create a range imager from an organised point cloud (throws exception
    // if point cloud is not organised), and a focal vector into the scene
    // (not required to be normalised) plus a focal length (only needed when
    // reconverting a range image back into a point cloud).
    RangeImager( const PointCloud::Ptr pcloud, const Vector3d &f, float focLen)
                            throw (PointCloudException, InvalidVectorException);

    inline float getFocalLength() const { return focLen;}

    // Get this object's range data matrix.
    cv::Mat getRangeData() const;

    // Get this object's current range data grey scale image (may be quadratic or linear).
    cv::Mat getRangeImage() const;

    // Return the maximum distance as currently used for the range image.
    float getMaxRange() const { return m_maxDist;}

    // Create a new point cloud derived from the range data assuming given camera focal length (ctor).
    // Focal vector for this point cloud is Z unit vector and up is Y unit vector. If current range
    // image is quadratic, it is first reset to be linear to allow for correct reconversion.
    PointCloud::Ptr createPointCloud();

    void reset();   // Reproduces linear range image.

    cv::Mat operator()() const; // Synonymous with getRangeData();

    // Produce a new grey scale (single channel) image of the range data with pixel
    // values scaled linearly with distance between 0 (distant) and 255 (close).
    // Maximum distances < 1 will be set to 1.
    cv::Mat createRangeImage( float maxDist);

    // Produce a new grey scale (single channel) image of the range data with pixel
    // values scaled as the square of the distance between 0 (distant) and 255 (close).
    // Quadratic (instead of linear) scaling exactly compensates for the "white-washing" effect of
    // normal linear range images where near distance changes are less apparent due to the
    // perspective effect of quadratically increasing the apparent size of surfaces as
    // they get closer to the centre of projection.
    // Maximum distances < 1 will be set to 1.
    cv::Mat createQuadraticRangeImage( float maxDist);

    inline bool isQuadratic() const { return isQuad;} // True iff stored range image is of the quadratic variety
    inline bool isLinear() const { return !isQuad;}  // True iff stored range image is of the linear variety

    static const float NO_DISTANCE_VAL; // Flag for pixels having no distance value

private:
    PointCloud::Ptr pcloud;
    Vector3d f;
    float m_maxDist;
    cv::Mat rngData;
    cv::Mat greyImg;
    bool isQuad;
    float focLen;

    // Create the single precision floating point single channel range data (CV_32FC1) matrix (rngData).
    // Pixels having no distance information are set to NO_DISTANCE_VAL.
    void createRangeData() throw (InvalidVectorException);

    cv::Mat createGenericRangeImage( float scale, uchar (*rangeVal)( float scale, float r));
};  // end class RangeImager

}   // end namespace

#endif
