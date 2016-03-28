#include <cstdlib>
#include <iostream>
using namespace std;

#include <LaplacianZC.h>
using namespace rlib::ImageProcess;
#include <opencv2/opencv.hpp>



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

    LaplacianZC lap( 5);

    cv::Mat lapImg = lap.getLaplacianImage( mbimg);
    cv::namedWindow("Laplacian");
    cv::imshow("Laplacian", lapImg);

    double min, max;
    cv::minMaxLoc( lapImg, &min, &max);
    cv::Mat lapImg2;
    lapImg.convertTo( lapImg2, CV_8U, -255./max, 255);
    cv::namedWindow("Laplacian Converted");
    cv::imshow("Laplacian Converted", lapImg2);

    cv::Mat lapDiff = lapImg - lapImg2;
    cv::namedWindow("Laplacian Difference");
    cv::imshow("Laplacian Difference", lapDiff);

    cv::Mat tlapImg;
    cv::threshold( lapDiff, tlapImg, 10, 255, cv::THRESH_BINARY);
    cv::namedWindow("Laplacian Thresholded");
    cv::imshow("Laplacian Thresholded", tlapImg);

    cv::Mat element5( 3, 3, CV_8U, cv::Scalar(1));
    cv::Mat opened;
    cv::morphologyEx( tlapImg, opened, cv::MORPH_OPEN, element5);
    cv::namedWindow("Laplacian Thresholded & Opened");
    cv::imshow("Laplacian Thresholded & Opened", opened);

    /*
    cv::Mat zcImg = lap.getZeroCrossings( tlapImg, 500.0);
    cv::namedWindow("Zero-crossings");
    cv::imshow("Zero-crossings", zcImg);
    */

    cv::waitKey();

    return EXIT_SUCCESS;
}   // end main
