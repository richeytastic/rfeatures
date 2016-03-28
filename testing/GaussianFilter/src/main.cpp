#include <FeatureUtils.h>
#include <iostream>
#include <cstdlib>


int main( int argc, char** argv)
{
    if ( argc != 3)
    {
        std::cerr << "Provide width and height" << std::endl;
        return EXIT_FAILURE;
    }   // end if

    const cv::Size imgSz( strtof( argv[1],0), strtof( argv[2], 0));
    const cv::Mat_<float> gf = RFeatures::make2DGaussian( imgSz);
    RFeatures::showImage( RFeatures::convertForDisplay( gf, true), "2D Gaussian", true);
    return EXIT_SUCCESS;
}   // end main

