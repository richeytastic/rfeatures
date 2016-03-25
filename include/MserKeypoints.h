/**
 * Detect blobs in images. See:
 * http://en.wikipedia.org/wiki/Maximally_stable_extremal_regions
 **/

#pragma once
#ifndef RFEATURES_MSER_KEYPOINTS_H
#define RFEATURES_MSER_KEYPOINTS_H

#include "KeypointsDetector.h"
using namespace RFeatures;


namespace RFeatures
{

class rFeatures_EXPORT MserKeypoints : public KeypointsDetector
{
public:
    MserKeypoints( const cv::Mat &originalImage,
            int delta,
            int minArea,
            int maxArea,
            double maxVariation,
            double minDiversity,
            int maxEvolution,
            double areaThreshold,
            double minMargin,
            int edgeBlurSize);

    virtual ~MserKeypoints(){}

    virtual Keypoints find() const;

private:
    cv::Mat working_image;
    int delta;
    int minArea;
    int maxArea;
    double maxVar;
    double minDiv;
    int maxEv;
    double areaThsh;
    double minMargin;
    int edgeBlurSz;
};  // end class MserKeypoints

}   // end namespace RFeatures

#endif



