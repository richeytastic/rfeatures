#include <cstdlib>
#include <cmath>
#include <iostream>
#include <sstream>
using namespace std;

#include "ImagePyramid.h"
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

    ImagePyramid pyramid( image, 2, 5);
    for ( int i = 0; i < pyramid.size(); ++i)
    {
        ostringstream oss;
        oss << "Image level " << i;
        showImage( oss.str(), *pyramid.getImage( i), true);
    }   // end for

    return EXIT_SUCCESS;
}   // end main

