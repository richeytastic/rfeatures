#include <cstdlib>
#include <cmath>
#include <iostream>
using namespace std;

#include "HOG_Operator.h"
using namespace RFeatures;


cv::Mat loadImage( int argc, char* argv[])
{
    if ( argc < 2)
    {
        cerr << "Please provide the image filename." << endl;
        exit( EXIT_FAILURE);
    }   // end if

    bool bw = false;
    if ( argc > 2)
        bw = true;

    cv::Mat image = cv::imread(argv[1], !bw);
    if ( image.empty())
    {
        cerr << "No image loaded!" << endl;
        exit( EXIT_FAILURE);
    }   // end if

    return image;
}   // end loadImage



void showImage( const string &title, const cv::Mat &img, bool wait=false)
{
    cv::namedWindow( title.c_str());
    cv::imshow( title.c_str(), img);
    if ( wait)
        cv::waitKey();
}   // end showImage



int main( int argc, char **argv)
{
    cv::Mat image = loadImage( argc, argv);

    cerr << "Image rows: " << image.rows << endl;
    cerr << "Image cols: " << image.cols << endl;
    cerr << "Image channels: " << image.channels() << endl;
    cerr << "Image row step: " << image.step << endl;

    showImage( "Original", image);

    HOG_Operator hogOp( image, 16, 18);

    showImage( "Original (Sqrt Gamma Corrected)", image);

    cv::Mat himg = hogOp.getHorizontalGradientImage();
    showImage( "Horizontal gradients", himg);
    cv::Mat vimg = hogOp.getVerticalGradientImage();
    showImage( "Vertical gradients", vimg);

    cv::Mat cimg( image.rows, image.cols, CV_8UC1, cv::Scalar(0));
    for ( int j = 0; j < image.rows; ++j)
        for ( int i = 0; i < image.cols; ++i)
            cimg.at<uchar>(j,i) = sqrt(pow( (double)himg.at<uchar>(j,i), 2) + pow( (double)vimg.at<uchar>(j,i), 2));
    showImage( "Gradient magnitudes", cimg);

    cv::Mat hogImg = hogOp.getHOGsImage();
    showImage( "HOG edges", hogImg);

    cv::waitKey();
    
    /*
    cv::imwrite( "h_gradients.png", himg);
    cv::imwrite( "v_gradients.png", vimg);
    cv::imwrite( "hv_gradients.png", cimg);
    cv::imwrite( "hog_descriptor.png", hogImg);
    */

    return EXIT_SUCCESS;
}   // end main



