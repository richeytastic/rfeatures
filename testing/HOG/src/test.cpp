#include <cstdlib>
#include <cmath>
#include <iostream>
using namespace std;
#include <cassert>

#include "HOG.h"
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


int roundMult( int v, int m)
{
    return (int)(v+m/2) / m * m;
}   // end roundMult


int main( int argc, char **argv)
{
    cv::Mat image = loadImage( argc, argv);

    cout << "Image rows: " << image.rows << endl;
    cout << "Image cols: " << image.cols << endl;
    cout << "Image channels: " << image.channels() << endl;
    cout << "Image row step: " << image.step << endl;

    showImage( "Original", image);

    const uint CELLSZ = 8;
    const uint NBINS = 18;

    cv::Mat rimage;
    // Scale image size to nearest multiple of CELLSZ
    cv::Size dsz( roundMult( image.cols, CELLSZ), roundMult( image.rows, CELLSZ));
    cv::resize( image, rimage, dsz);
    cout << "Resized image rows: " << rimage.rows << endl;
    cout << "Resized image cols: " << rimage.cols << endl;

    HOG hog( rimage, CELLSZ, NBINS);

    cv::Mat hogs = hog();
    showImage( "Original (resized and sqrt Gamma Corrected)", rimage);
    cv::Mat hogImg = HOG::createVisualisation( hogs, CELLSZ);
    showImage( "HOG edges", hogImg);
    while ( cv::waitKey() != 27);   // Wait for escape to exit
    
    return EXIT_SUCCESS;
}   // end main



