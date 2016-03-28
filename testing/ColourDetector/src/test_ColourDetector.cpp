#include <ColourDetector.h>
using namespace rlib::ImageProcess;
#include <cstdlib>
#include <opencv2/opencv.hpp>


int main()
{
    ColourDetector cd;
    cv::Mat image = cv::imread("img0.jpeg");
    if ( image.empty())
        return EXIT_FAILURE;
    cd.setTargetColour(180,190,255);
    cd.setColourDistanceThreshold( 60);
    cv::namedWindow("result");
    cv::imshow("result", cd.process(image));
    cv::waitKey();
    return EXIT_SUCCESS;
}   // end main
