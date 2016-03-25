#include "MserKeypoints.h"
using namespace RFeatures;


MserKeypoints::MserKeypoints( const cv::Mat &img, int d, int minA, int maxA, double maxV,
    double minD, int maxE, double areaT, double minM, int blSz)
    : KeypointsDetector( img),
    delta(d), minArea(minA), maxArea(maxA), maxVar(maxV), minDiv(minD),
    maxEv(maxE), areaThsh(areaT), minMargin(minM), edgeBlurSz(blSz)
{
    working_image = img.clone();
}   // end ctor



Keypoints MserKeypoints::find() const
{
    cv::MserFeatureDetector fd( delta, minArea, maxArea, maxVar, minDiv,
                                  maxEv, areaThsh, minMargin, edgeBlurSz);
    return KeypointsDetector::detectKeypoints( fd, working_image);
}   // end find
