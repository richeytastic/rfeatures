#include <CircleFeature.h>
using RFeatures::CircleFeature;
#include <FeatureUtils.h>
#include <cassert>
#include <sstream>
#include <iostream>
using std::cerr;
using std::endl;
#include <cstdlib>
typedef unsigned char byte;


cv::Mat loadImage( const string fname)
{
    cv::Mat img = cv::imread( fname, false);   // Grey scale
    if ( img.channels() == 3)
    {
        cv::Mat greyImg;
        cv::cvtColor( img, greyImg, CV_BGR2GRAY);
        img = greyImg;
    }   // end if
    assert( img.channels() == 1);
    return img;
}   // end loadImage


void printVec( int i, int j, const cv::Mat_<float> v)
{
    cerr << "Row,col (" << i << "," << j << ") :";
    for ( int k = 0; k < v.total(); ++k)
        cerr << " " << v.ptr<float>(0)[k];
    cerr << endl;
}   // end printVec


int main( int argc, char** argv)
{
    if ( argc != 2)
    {
        cerr << "Provide image filename" << endl;
        return EXIT_FAILURE;
    }   // end if

    string fname = argv[1];
    const cv::Mat img = loadImage( fname);
    fname = fname.substr( fname.find_last_of("/") + 1);

    const cv::Size imgSz = img.size();
    const cv::Size knSz(imgSz.width/16, imgSz.height/16);
    cerr << "Feature size (height,width) = " << knSz << endl;

    CircleFeature cf( img, 9);
    RFeatures::showImage( cf.showPattern( imgSz), "Circles", true);

    cv::Mat_<byte> outImg = cv::Mat_<byte>::zeros( imgSz);

    cv::Rect rct(0,0,knSz.width,knSz.height);
    for ( ; (rct.y + rct.height - 1) < imgSz.height; rct.y += knSz.height/2)
    {
        rct.x = ( rct.y % knSz.height != 0) ? knSz.width/2 : 0;
        for ( ; (rct.x + rct.width - 1) < imgSz.width; rct.x += knSz.width)
        {
            const cv::Mat_<float> v = cf.calcFeature(rct);    // Calc the feature on this patch
            outImg(rct) |= cf.visFeature(v, knSz);
        }   // end for
    }   // end for

    RFeatures::showImage( img, "Original grey level (" + fname +")", false);
    RFeatures::showImage( outImg, "Features image", true);

    return EXIT_SUCCESS;
}   // end main
