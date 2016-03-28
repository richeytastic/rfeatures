#include <cstdlib>
#include <iostream>
using namespace std;

#include <Histogram1D.h>
using namespace rlib::ImageProcess;
#include <CannyOperator.h>
#include <HoughLinesOperator.h>
using namespace RFeatures;



int main( int argc, char **argv)
{
    cv::Mat image = cv::imread(argv[1], 0); // 0 2nd param is open b&w
    if ( image.empty())
    {
        cerr << "No image loaded!" << endl;
        return EXIT_FAILURE;
    }   // end if

    //cv::namedWindow("Original");
    //cv::imshow("Original", image);

    //cv::Mat eqlImg = Histogram1D::equalise( image);

    cv::GaussianBlur( image, image, cv::Size(3,3), 1.5);
    cv::medianBlur( image, image, 3);

    CannyOperator co( image, 60, 180);
    cv::Mat houghImg = HoughLinesOperator( co, 34, 30, 10)();

    cv::namedWindow("Hough Lines");
    cv::imshow("Hough Lines", houghImg);

    cv::waitKey();

    return EXIT_SUCCESS;
}   // end main
