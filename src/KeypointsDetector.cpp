#include "KeypointsDetector.h"
using namespace RFeatures;


KeypointsDetector::KeypointsDetector( const cv::Mat &img)
{
    original_image = img.clone();
}   // end ctor


KeypointsDetector::~KeypointsDetector()
{
    original_image.release();
}   // end dtor


cv::Mat KeypointsDetector::draw( const Keypoints &kps, cv::Scalar col, cv::Mat *img) const
{
    bool useProvided( img != NULL);
    cv::Mat outImg;
    if ( !useProvided)
        outImg = original_image.clone();

    cv::drawKeypoints( original_image, kps,
            ( useProvided ? *img : outImg), col,
            cv::DrawMatchesFlags::DRAW_OVER_OUTIMG | keypointDrawFlag());
    return ( useProvided ? *img : outImg);
}   // end draw


cv::Mat KeypointsDetector::draw( const Keypoints &kps, cv::Mat *img, cv::Scalar col) const
{
    return draw( kps, col, img);
}   // end draw


cv::Mat KeypointsDetector::operator()() const
{
    return this->draw( this->find());
}   // end operator()


Keypoints KeypointsDetector::detectKeypoints( cv::FeatureDetector &fd, const cv::Mat &img) const
{
    Keypoints keypoints;    // vector<cv::Keypoint> --> Keypoints
    fd.detect( img, keypoints);
    return keypoints;
}   // end detectKeypoints


int KeypointsDetector::keypointDrawFlag() const
{
    return cv::DrawMatchesFlags::DEFAULT;   // 0
}   // end keypointDrawFlag


cv::Mat KeypointsDetector::cloneOriginalImage() const
{
    return original_image.clone();
}   // end cloneOriginalImage
