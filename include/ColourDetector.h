#pragma once
#ifndef RFEATURES_COLOUR_DETECTOR_H
#define RFEATURES_COLOUR_DETECTOR_H

#include "RFeatures.h"

namespace RFeatures
{

class rFeatures_EXPORT ColourDetector
{
public:
    ColourDetector();
    ColourDetector( float minDist, uchar r, uchar g, uchar b);
    ColourDetector( float minDist, const cv::Vec3b &targetCol);

    void setColourDistanceThreshold( float minDist);
    inline float getColourDistanceThreshold() const { return m_minDist;}

    void setTargetColour( uchar red, uchar green, uchar blue);
    void setTargetColour( const cv::Vec3b &col);    // Col should be BGR ordered
    inline cv::Vec3b getTargetColour() const { return m_target;}

    cv::Mat process( const cv::Mat &img);

private:
    float m_minDist;  // Colour tolerance
    cv::Vec3b m_target; // Target colour
    cv::Mat m_converted;    // Input image converted to CIE L*a*b* colour space
    cv::Mat m_result;   // Resulting image

    // Get normalised city block distance from provided colour to target colour.
    int getNormDistanceCityBlock( const cv::Vec3b&) const;
    // Get normalised Euclidean distance from provided colour to target colour.
    int getNormDistanceEuclidean( const cv::Vec3b&) const;
};  // end class ColourDetector

}   // end namespace RFeatures

#endif
