/**
 * Encapsulate functionality of different feature/keypoint detectors.
 * Richard Palmer
 * June 2012
 **/

#pragma once
#ifndef RFEATURES_IMAGE_MODIFIER_H
#define RFEATURES_IMAGE_MODIFIER_H

//#define INCLUDE_OPENCV_NONFREE

#include "rFeatures_Export.h"
#include <vector>
using std::vector;
#include <opencv2/opencv.hpp>
#ifdef INCLUDE_OPENCV_NONFREE
    #include <opencv2/nonfree/nonfree.hpp>  // For SIFT and SURF
#endif
#include <boost/shared_ptr.hpp>
typedef unsigned int uint;
typedef vector<cv::KeyPoint> Keypoints;
typedef vector<cv::Vec4i> Lines;
typedef vector<cv::Vec3f> Circles;


namespace RFeatures
{

class rFeatures_EXPORT ImageModifier
{
public:
    explicit ImageModifier( const cv::Mat &img);
    inline cv::Mat getImage() const { return image;}    // The image used for this image modifier

    // Detect keypoints (or lines or circles). In all cases, the parameter vector is appended to.
    void detectHarrisKeypoints( Keypoints&) const;
    void detectFastKeypoints( Keypoints&) const;

#ifdef INCLUDE_OPENCV_NONFREE
    void detectSiftKeypoints( Keypoints&) const;
    void detectSurfKeypoints( Keypoints&) const;
#endif

    void detectStarKeypoints( Keypoints&) const;
    void detectMserKeypoints( Keypoints&) const;
    void detectHoughLines( int lowCanny, int highCanny, int minVote, int minLen,
                           int maxGap, int accDist, double accAngle, Lines &lns) const;
    void detectHoughCircles( Circles &) const;

    // Draw the provided features onto the current image and return the image ready for display.
    cv::Mat drawFeatures( const Keypoints* kps=NULL, const Lines* lns=NULL, const Circles* circles=NULL) const;

private:
    cv::Mat image;  // The working image
};  // end class

}   // end namespace

#endif


