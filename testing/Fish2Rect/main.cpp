#include "Fish2Rect.h"
#include <cstdlib>
#include <iostream>
#include <string>


int main( int argc, char** argv)
{
    using namespace std;
    if ( argc < 2)
    {
        cerr << "Usage: " << argv[0] << " image_file [pixels_focal_length]" << endl;
        cerr << "If focal length is not supplied, fisheye retification uses radial parameters for AAM camera 3" << endl;
        return EXIT_FAILURE;
    }   // end if

    const string fname = argv[1];
    double focalLen = 0;
    if ( argc > 2)
        focalLen = strtod( argv[2], 0);

    const cv::Mat img = cv::imread( fname, true);
    const cv::Size newSz( img.cols/2, img.rows/2);

    cv::Mat dimg;

    /*
    cv::resize( img, dimg, newSz);
    cv::namedWindow( "In Image");
    cv::imshow( "In Image", dimg);
    cv::waitKey();
    */

    cv::Mat rimg;
    if ( argc > 2)
        rimg = RFeatures::Fish2Rect::rectify( img, focalLen);
    else
    {
        const double R = 600;
        const double A3 = -0.0000003226415;  // Radial A3 - Camera 3
        const double A5 = -0.000000000000302337; // Radial A5 - Camera 3
        rimg = RFeatures::Fish2Rect::rectify(img, R, A3, A5);
    }   // end else

    cv::resize( rimg, dimg, newSz);

    cv::namedWindow( "Rectified");
    cv::imshow( "Rectified", dimg);
    while ( cv::waitKey() != 27);

    return EXIT_SUCCESS;
}   // end main




