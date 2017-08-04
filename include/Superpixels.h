#ifndef RFEATURES_SUPERPIXELS_H
#define RFEATURES_SUPERPIXELS_H

#ifdef _WIN32
#pragma warning( disable : 4251)
#endif

#include "SLIC.h"
#include <Random.h> // rlib
#include <opencv2/opencv.hpp>
#include <boost/unordered_map.hpp>
typedef unsigned int uint;
typedef unsigned char byte;


namespace RFeatures
{

class rFeatures_EXPORT Superpixels
{
public:
    enum SPType
    {
        SP_COUNT,
        SP_PIXELS
    };  // end enum

    // Provided image must be either CV_8UC3 or CV_8UC1.
    // spParam is either the number of superpixels (if spFlag = SP_COUNT)
    // or the number of pixels in a superpixel (if spFlag = SP_PIXELS).
    // By default spParam is set to 200 and used as the required number of superpixels.
    // compactness is the compactness factor which acts to weight the distance
    // measure of each pixel from its parent superpixel - good values are between 0 and 40.
    Superpixels( const cv::Mat &img, int spParam=200, SPType spFlag=SP_COUNT, int compactness=10);
    virtual ~Superpixels();

    // Extract labels for the pixels.
    // Labels are placed into the array in scan order (i.e. from left to right, top to bottom).
    // The number of different labels (discrete superpixels) is set in out parameter numLabels.
    const int *const extract( int &numLabels);

    // Use the pixel labels to create a colour image (CV_8UC3) having a discrete colour per superpixel.
    cv::Mat_<cv::Vec3b> createLabelImage( rlib::Random&);

    // Return a single channel image (CV_8UC1) displaying the outlines of the superpixels in white on black.
    cv::Mat_<byte> drawOutlines();

private:
    const cv::Size imgSz_;
    const SPType spFlag_;
    int spParam_;
    int compactness_;
    SLIC* segment_;
    int numLabs_;
    uint* imgBuff_;
    int* klabs_;

    void extract(); // extract helper
};  // end class

}   // end namespace

#endif
