#include <cstdlib>
#include <iostream>
#include <string>
using std::string;
using std::cerr;
using std::endl;

#include <opencv2/opencv.hpp>
#include <CannyOperator.h>
#include <FeatureUtils.h>


int main( int argc, char **argv)
{
    if ( argc != 5)
    {
        cerr << "usage: " << argv[0] << " input_image low_thresh high_thresh output_image" << endl;
        return EXIT_FAILURE;
    }   // end if

    cv::Mat image = cv::imread(argv[1], 0); // 0 2nd param is open b&w
    const int lowThresh = strtol( argv[2], 0, 10);
    const int highThresh = strtol( argv[3], 0, 10);
    const string outfname = argv[4];
    if ( image.empty())
    {
        cerr << "No image loaded!" << endl;
        return EXIT_FAILURE;
    }   // end if

    //cv::GaussianBlur( image, image, cv::Size(3,3), 1.5);
    //cv::medianBlur( image, image, 3);

    //cv::namedWindow("Original");
    //cv::imshow("Original", image);

    cv::Mat_<float> cannyImg = RFeatures::CannyOperator( image, lowThresh, highThresh).getEdgeImage();
    cv::Mat dimg = RFeatures::convertForDisplay( cannyImg);

    RFeatures::saveImage( outfname, dimg);

    return EXIT_SUCCESS;
}   // end main
