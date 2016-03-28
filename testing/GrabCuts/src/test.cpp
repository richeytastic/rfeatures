#include <cstdlib>
#include <iostream>
using namespace std;

#include <GrabCutsOperator.h>
using namespace RFeatures;



int main( int argc, char **argv)
{
    cv::Mat image = cv::imread(argv[1]);
    if ( image.empty())
    {
        cerr << "No image loaded!" << endl;
        return EXIT_FAILURE;
    }   // end if

    //cv::Mat imageGB;
    //cv::GaussianBlur( image, imageGB, cv::Size(5,5), 1.5);
    cv::medianBlur( image, image, 3);

    // Foreground area set as a wide band across the image with slim
    // bands of pixels missing from the top and bottom of the image.
    cv::Rect fgArea( 0, 100, image.cols, image.rows - 150);

    // Get the foreground image
    cv::Mat gcImg = GrabCutsOperator( image, fgArea, 4)();

    // Draw the foregrond rectangle on the original image
    cv::rectangle( image, fgArea, cv::Scalar(0,0,255), 2);

    // Show original image (with foreground rectangle drawn)
    cv::namedWindow("Original");
    cv::imshow("Original", image);

    // Show foreground image
    cv::namedWindow("Foreground");
    cv::imshow("Foreground", gcImg);

    cv::waitKey();

    return EXIT_SUCCESS;
}   // end main
