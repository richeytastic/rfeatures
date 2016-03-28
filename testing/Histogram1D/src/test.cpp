#include <Histogram1D.h>

#include <cstdlib>
#include <iostream>
using namespace std;

#include <opencv2/opencv.hpp>


int main( int argc, char **argv)
{
    if ( argc == 1)
    {
        cerr << "Supply image filename" << endl;
        return EXIT_FAILURE;
    }   // end if

    cv::Mat image = cv::imread(argv[1], 0); // 0 2nd param is open b&w
    if ( image.empty())
    {
        cerr << "No image loaded!" << endl;
        return EXIT_FAILURE;
    }   // end if

    cv::namedWindow("Original");
    cv::imshow("Original", image);

    RFeatures::Histogram1D h;

    cv::Mat_<float> fimg;
    image.convertTo( fimg, CV_32F);
    cv::log( fimg, fimg);
    cv::Mat img;
    fimg.convertTo( img, CV_8U);
    cv::Mat histImg1 = h.getHistogramImage( img);
    cv::namedWindow("Histogram1D (LOG) - Original");
    cv::imshow("Histogram1D (LOG) - Original", histImg1);

    /*
    cv::Mat eqlImg = RFeatures::Histogram1D::equalise( image);
    cv::namedWindow("Equalised Image");
    cv::imshow("Equalised Image", eqlImg);

    cv::Mat histImg2 = h.getHistogramImage( eqlImg);
    cv::namedWindow("Histogram1D - Stretched");
    cv::imshow("Histogram1D - Stretched", histImg2);
    */

    cv::waitKey();

    return EXIT_SUCCESS;
}   // end main
