#include <AdaptiveDepthMinFilter.h>
using RFeatures::AdaptiveDepthMinFilter;
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
    if ( argc < 4)
    {
        cerr << "Provide image filename (to convert to CV_32FC1 depth) and real patch size (width and height) in metres" << endl;
        return EXIT_FAILURE;
    }   // end if

    const string fname = argv[1];
    cv::Mat img;
    RFeatures::loadImage( fname, img, true);    // Load img as black and while
    assert( img.channels() == 1);

    cv::Mat_<float> depthMap;
    img.convertTo( depthMap, CV_32FC1);
    const cv::Size2f realPatchSize( strtof( argv[2], 0), strtof( argv[3], 0));

    AdaptiveDepthMinFilter filter( depthMap, realPatchSize);
    const cv::Mat_<float> filteredImg = filter.filter();
    const cv::Mat dimg = RFeatures::convertForDisplay( filteredImg, false);

    RFeatures::showImage( RFeatures::convertForDisplay( depthMap, true), "Original", false);
    RFeatures::showImage( dimg, "Min filtered", true);
    return EXIT_SUCCESS;
}   // end main

