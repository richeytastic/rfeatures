#include <cstdlib>
#include <cmath>
#include <iostream>
#include <sstream>
using namespace std;

#include "ImagePyramid.h"
using namespace RFeatures;

#include <CpuTimer.h>


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
    cv::Mat img = loadImage( argc, argv);

    cerr << "Image rows: " << img.rows << endl;
    cerr << "Image cols: " << img.cols << endl;
    cerr << "Image channels: " << img.channels() << endl;
    cerr << "Image row step: " << img.step << endl;

    showImage( "Original", img);

    ImagePyramid *pyramid = NULL;
    { rlib::CpuTimer ct("Image Pyramid timer", cerr);
        pyramid = new ImagePyramid( img, 5, 20, 40);
    }   // end timer

    for ( int i = 0; i < pyramid->size(); ++i)
    {
        double scale = pyramid->getScale(i);
        ostringstream oss;
        oss << "Image level " << i;
        cout << oss.str() << " scale = " << scale;
        cv::Mat pimg = pyramid->getImage( i);
        cout << " dims = (" << pimg.rows << ", " << pimg.cols << ")" << endl;
        showImage( oss.str(), pimg, true);
    }   // end for

    return EXIT_SUCCESS;
}   // end main

