#pragma once
#ifndef RFEATURES_KEYPOINTS_DETECTOR_H
#define RFEATURES_KEYPOINTS_DETECTOR_H

#include "RFeatures.h"


namespace RFeatures
{
class rFeatures_EXPORT KeypointsDetector
{
public:
    KeypointsDetector( const cv::Mat &originalImage);
    virtual ~KeypointsDetector();

    // Find the keypoints in the provided image.
    virtual Keypoints find() const = 0;

    // Draw the provided keypoints over the provided image. If outImg is NULL, keypoints will
    // be drawn over cloned version of original image. Default colour is white.
    virtual cv::Mat draw( const Keypoints &keypoints,
            cv::Scalar colour=cv::Scalar(255,255,255), cv::Mat *outImg=NULL) const;

    virtual cv::Mat draw( const Keypoints &keypoints, cv::Mat *outImg,
            cv::Scalar colour=cv::Scalar(255,255,255)) const;

    // Synonymous with this->draw( this->find()) unless overridden.
    virtual cv::Mat operator()() const;

protected:
    Keypoints detectKeypoints( cv::FeatureDetector &fd, const cv::Mat &workingImage) const;
    cv::Mat cloneOriginalImage() const;

    // Return the flag for drawing keypoints. Alternatives that
    // may be implemented by subclasses are returning 2 or 4:
    // cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS (2), or
    // cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS (4)
    virtual int keypointDrawFlag() const;

private:
    cv::Mat original_image;
};  // end class KeypointsDetector

}   // end namespace RFeatures

#endif
