#include <AdaptiveDepthSegmenter.h>
using RFeatures::AdaptiveDepthSegmenter;
#include <FeatureUtils.h>
#include <cassert>
#include <cstdlib>
#include <iostream>
using std::cerr;
using std::endl;
#include <string>
using std::string;


int main( int argc, char** argv)
{
    if ( argc < 5)
    {
        cerr << "Provide image filename (to convert to CV_32FC1 depth), and real patch size (width and height) in metres, and threshold" << endl;
        return EXIT_FAILURE;
    }   // end if

    const string fname = argv[1];
    cv::Mat img;
    RFeatures::loadImage( fname, img, true);    // Load img as black and while
    assert( img.channels() == 1);

    cv::Mat_<float> depthMap;
    img.convertTo( depthMap, CV_32FC1);
    const cv::Size2f realPatchSize( strtof( argv[2], 0), strtof( argv[3], 0));
    const float tval = strtof(argv[4], 0);

    AdaptiveDepthSegmenter ads( depthMap, realPatchSize, tval);
    const cv::Mat_<byte> filteredImg = ads.filter();

    RFeatures::showImage( RFeatures::convertForDisplay( depthMap, true), "Original", false);
    RFeatures::showImage( filteredImg, "Result", true);

    return EXIT_SUCCESS;
}   // end main

