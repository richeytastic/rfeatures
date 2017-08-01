#ifndef RFEATURES_SOBEL_MAKER_H
#define RFEATURES_SOBEL_MAKER_H

#include <opencv2/opencv.hpp>
#include "rFeatures_Export.h"

namespace RFeatures
{

class rFeatures_EXPORT SobelMaker
{
public:
    explicit SobelMaker( const cv::Mat& img);   // img must be single channel of any depth

    // Returned maps have values in range [0,1]
    cv::Mat_<float> makeSobelD1( int ksize=3) const;
    cv::Mat_<float> makeSobelD2( int ksize=3) const;

private:
    cv::Mat_<float> _img;
};  // end class


}   // end namespace

#endif
