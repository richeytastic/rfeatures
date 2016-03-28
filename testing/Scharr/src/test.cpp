#include <cstdlib>
#include <iostream>
using namespace std;

#include <opencv2/opencv.hpp>
#include <cmath>


void doScharr( cv::Mat &img, uchar thresholdVal)
{
    cv::Mat scharrX;
    cv::Mat scharrY;
    cv::Scharr( img, scharrX, CV_16S, 1, 0, 3);
    cv::Scharr( img, scharrY, CV_16S, 0, 1, 3);
    cv::Mat scharr = abs(scharrX) + abs(scharrY);   // L1 norm

    double min, max;
    cv::minMaxLoc( scharr, &min, &max);
    // Convert to 8-bit image
    cv::Mat scharrImg;
    scharr.convertTo( scharrImg, CV_8U, -255./max, 255);

    cv::Mat scharrThresholded;
    cv::threshold( scharrImg, scharrThresholded, thresholdVal, 255, cv::THRESH_BINARY);
    cv::namedWindow("Scharr Thresholded");
    cv::imshow("Scharr Thresholded", scharrThresholded);
}   // end doScharr


int main( int argc, char **argv)
{
    cv::Mat image = cv::imread(argv[1], 0); // 0 2nd param is open b&w
    if ( image.empty())
    {
        cerr << "No image loaded!" << endl;
        return EXIT_FAILURE;
    }   // end if

    cv::namedWindow("Original");
    cv::imshow("Original", image);

    cv::Mat mbimg;
    cv::medianBlur( image, mbimg, 5);

    doScharr( mbimg, 220);

    /*
    cv::Mat mbimg;
    cv::medianBlur( image, mbimg, 3);
    cv::namedWindow("Median Blurred");
    cv::imshow("Median Blurred", mbimg);

    cv::Mat shimg;
    cv::GaussianBlur( mbimg, shimg, cv::Size(0, 0), 3);
    cv::addWeighted( mbimg, 1.5, shimg, -0.5, 0, shimg);

    cv::namedWindow("Sharpened Image");
    cv::imshow("Sharpened Image", shimg);

    cv::Mat eqlImg;
    cv::equalizeHist( mbimg, eqlImg);
    cv::namedWindow("Equalised Image");
    cv::imshow("Equalised Image", eqlImg);
    */

    cv::waitKey();

    return EXIT_SUCCESS;
}   // end main
