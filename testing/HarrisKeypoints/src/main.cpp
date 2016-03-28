#include <cstdlib>
#include <iostream>
using namespace std;

#include <HarrisKeypoints.h>
using namespace RFeatures;



int main( int argc, char **argv)
{
    if ( argc != 2)
    {
        cerr << "Provide an image file" << endl;
        return EXIT_FAILURE;
    }   // end if

    cv::Mat image = cv::imread(argv[1]);
    if ( image.empty())
    {
        cerr << "No image loaded!" << endl;
        return EXIT_FAILURE;
    }   // end if

    cv::Mat fdImg = HarrisKeypoints( image, 200, 0.1, 10)();
    cv::namedWindow("fd Keypoints");
    cv::imshow("fd Keypoints", fdImg);

    cv::waitKey();

    return EXIT_SUCCESS;
}   // end main
