#include <cstdlib>
#include <iostream>
using namespace std;

#include <Histogram1D.h>
using namespace rlib::ImageProcess;
#include <HoughCirclesOperator.h>
using namespace RFeatures;



int main( int argc, char **argv)
{
    cv::Mat image = cv::imread(argv[1]);
    if ( image.empty())
    {
        cerr << "No image loaded!" << endl;
        return EXIT_FAILURE;
    }   // end if


    cv::Mat imageGB;
    cv::cvtColor( image, imageGB, CV_BGR2GRAY);
    cv::GaussianBlur( imageGB, imageGB, cv::Size(5,5), 1.5);

    HoughCirclesOperator hco( imageGB, 130, 35, 40, 7, 100);
    HoughCirclesOperator::drawCircles( hco.findCircles(), image);

    cv::namedWindow("Hough Circles");
    cv::imshow("Hough Circles", image);

    cv::waitKey();

    return EXIT_SUCCESS;
}   // end main
