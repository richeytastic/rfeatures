#pragma once
#ifndef RFEATURES_INTENSITY_INDEXER_H
#define RFEATURES_INTENSITY_INDEXER_H

#include <opencv2/opencv.hpp>
typedef unsigned char byte;
#include "rFeatures_Export.h"


namespace RFeatures
{

class rFeatures_EXPORT IntensityIndexer
{
public:
    // Image must be either 3 channel or 1 channel CV_8U. 3 channel images
    // are first converted to CIE Lab space and the lightness channel only
    // is used. Once a single channel image is produced, intensity levels
    // are uniformly stretched to ensure that binning is proportional across
    // all intensity levels.
    IntensityIndexer( const cv::Mat& image, int numLevs=256);

    // Calculates the regions having the most frequent intensity values and maps
    // these back to the original image within the given number of banded levels.
    // Dims of mask (if given) must be same as width/height of subRect.
    cv::Mat_<byte> calcMapping( const cv::Rect subRect=cv::Rect(0,0,0,0), cv::Mat_<byte> mask=cv::Mat_<byte>()) const;
    cv::Mat_<byte> operator()( const cv::Rect subRect=cv::Rect(0,0,0,0), cv::Mat_<byte> mask=cv::Mat_<byte>()) const;

private:
    const int _numLevs;
    cv::Mat_<byte> _image;
};  // end class

}   // end namespace

#endif
