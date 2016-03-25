#pragma once
#ifndef RFEATURES_IMAGE_PYRAMID_H
#define RFEATURES_IMAGE_PYRAMID_H

#include <vector>
using std::vector;
#include <opencv2/opencv.hpp>
#include "rFeatures_Export.h"


namespace RFeatures
{

class rFeatures_EXPORT ImagePyramid
{
public:
    // The highest resolution image halucinates an image at twice the resolution of the
    // originally provided image. Parameter octaves specifies the number of whole halvings
    // of the image to be contained in the image pyramid. Parameter lambda controls the
    // number of levels between octaves (half resolution images) i.e. the number of images
    // you must go down in the image stack to find one at half the resolution.
    // Parameter minSize specifies the minimum allowed size for a dimension of an
    // image in the pyramid.
    // Leaving the default parameters gives an image pyramid with a single image
    // at twice the scale as the provided original.
    ImagePyramid( const cv::Mat &originalImage, int octaves=1, int lambda=1, int minSize=1);

    size_t size() const { return imgStack.size();}
    vector<cv::Mat> getImages() const { return imgStack;}
    vector<double> getScales() const { return scales;}

    // Return a single image in the pyramid. The highest resolution image is at
    // index 0 (which halucinates an image at twice the resolution of the
    // originally provided image). Returns an empty cv::Mat if idx is out of range.
    cv::Mat getImage( int idx) const;

    // Get the scale an image was produced at (relative to the original size of the image).
    // Returns <= 0 of provided index is invalid.
    double getScale( int idx) const;

private:
    vector<cv::Mat> imgStack;   // Highest resolution image at index 0 (twice power of original)
    vector<double> scales;
};  // end class ImagePyramid

}   // end namespace RFeatures

#endif
