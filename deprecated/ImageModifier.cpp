#include "ImageModifier.h"
#include "HarrisKeypoints.h"
#include "FastKeypoints.h"
#include "StarKeypoints.h"
#include "MserKeypoints.h"
#include "HoughLinesOperator.h"
#include "HoughCirclesOperator.h"
#include "CannyOperator.h"
#include "HOG.h"
using namespace RFeatures;


ImageModifier::ImageModifier( const cv::Mat &img) : image(img)
{}   // end ctor


void ImageModifier::detectHarrisKeypoints( Keypoints &kps) const
{
    Keypoints kps1 = HarrisKeypoints( image, 300, 0.01, 7).find();
    kps.insert(kps.end(), kps1.begin(), kps1.end());
}   // end detectHarrisKeypoints


void ImageModifier::detectFastKeypoints( Keypoints &kps) const
{
    Keypoints kps1 = FastKeypoints( image, 40).find();
    kps.insert(kps.end(), kps1.begin(), kps1.end());
}   // end detectFastKeypoints


#ifdef INCLUDE_OPENCV_NONFREE
void ImageModifier::detectSiftKeypoints( Keypoints &kps) const
{
    cv::SIFT sift(100);
    sift( image, cv::noArray(), kps, cv::noArray());
}   // end detectSiftKeypoints


void ImageModifier::detectSurfKeypoints( Keypoints &kps) const
{
    cv::SURF surf( 2500);
    surf( image, cv::noArray(), kps);
}   // end detectSurfKeypoints
#endif


void ImageModifier::detectStarKeypoints( Keypoints &kps) const
{
    Keypoints kps1 = StarKeypoints( image).find();
    kps.insert(kps.end(), kps1.begin(), kps1.end());
}   // end detectStarKeypoints


void ImageModifier::detectMserKeypoints( Keypoints &kps) const
{
    int delta = 0;
    int minArea = 0;
    int maxArea = 0;
    double maxVariation = 0;
    double minDiversity = 0;
    int maxEvolution = 0;
    double areaThreshold = 0;
    double minMargin = 0;
    int edgeBlurSize = 0;
    Keypoints mserKps = MserKeypoints( image, delta, minArea, maxArea, maxVariation, minDiversity,
                                    maxEvolution, areaThreshold, minMargin, edgeBlurSize).find();
    kps.insert( kps.end(), mserKps.begin(), mserKps.end());
}   // end detectMserKeypoints


void ImageModifier::detectHoughLines( int lowCanny, int highCanny, int minVote,
                                       int minLen, int maxGap, int accDist, double accAngle, Lines &lns) const
{
    CannyOperator co( image, lowCanny, highCanny);
    HoughLinesOperator( co, minVote, minLen, maxGap, accDist, accAngle).findLines( lns);
}   // end detectHoughLines


void ImageModifier::detectHoughCircles( Circles &circles) const
{
    cv::Mat imageGB;
    if ( image.channels() == 1)
        imageGB = image.clone();
    else
        cv::cvtColor( image, imageGB, CV_BGR2GRAY);  // Convert to gray scale first
    cv::GaussianBlur( imageGB, imageGB, cv::Size(3,3), 1.5);
    // Parameters to HoughCirclesOperator as follows:
    // 1) The image
    // 2) High Canny Threshold
    // 3) Minimum number of votes needed to register a circle
    // 4) Minimum distance allowed between circle centres
    // 5) Minimum allowed radius of a circle
    // 6) Maximum allowed radius of a circle
    HoughCirclesOperator( imageGB, 100, 35, 40, 3, 100).findCircles( circles);
}   // end detectHoughCircles



cv::Mat ImageModifier::drawFeatures( const Keypoints *kps, const Lines *lns, const Circles *circs) const
{
    cv::Mat outImg = cv::Mat::zeros( image.size(), CV_8UC3);
    if ( kps != NULL)
        cv::drawKeypoints( image, *kps, outImg, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_OVER_OUTIMG);
    if ( lns != NULL)
        HoughLinesOperator::drawLines( *lns, outImg);
    if ( circs != NULL)
        HoughCirclesOperator::drawCircles( *circs, outImg);

    cv::Mat img;
    if ( image.channels() == 1)
        cv::cvtColor( image, img, CV_GRAY2BGR);
    else
        img = image.clone();

    cv::Mat mask;   // Mask must be gray level
    cv::cvtColor( outImg, mask, CV_BGR2GRAY);

    outImg.copyTo( img, mask);
    return img;
}   // end drawFeatures

