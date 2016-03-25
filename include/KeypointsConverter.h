#pragma once
#ifndef RFEATURES_KEYPOINTS_CONVERTER_H
#define RFEATURES_KEYPOINTS_CONVERTER_H

#include "RFeatures.h"
#include <boost/foreach.hpp>


namespace RFeatures
{

class rFeatures_EXPORT KeypointsConverter
{
public:
    // Create a new point cloud from a bunch of 2D keypoints and an organised point cloud reference.
    KeypointsConverter( const Keypoints&, const cv::Mat_<cv::Vec3f>);

    // Extract an unordered list of points.
    cv::Mat_<cv::Vec3f> get3DKeypoints() const;

    // Returns only those keypoints with valid depth info
    Keypoints cullNullDepthKeypoints() const;

private:
    const cv::Mat_<cv::Vec3f> m_ref;
    const Keypoints m_kpts;
    byte m_r;    // Red colour
    byte m_g;    // Green colour
    byte m_b;    // Blue colour
}; // end class

}  // end namespace


#endif
