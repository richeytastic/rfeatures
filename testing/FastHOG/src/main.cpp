#include <FastHOG.h>
using RFeatures::FastHOG;
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
    return img;
}   // end loadImage



int main( int argc, char** argv)
{
    if ( argc < 2)
    {
        cerr << "Provide image filename(s)" << endl;
        return EXIT_FAILURE;
    }   // end if

    const int nbins = 9;
    const cv::Size knSz(5,5);

    cv::Size imgSz;
    vector<cv::Mat> angleImgs, magImgs;
    cerr << "HOGs using Sobel on grey image(s)" << endl;
    for ( int i = 1; i < argc; ++i)
    {
        string fname = argv[i];
        const cv::Mat img = loadImage( fname);
        fname = fname.substr( fname.find_last_of("/") + 1);
        imgSz = img.size();
        cv::Mat_<float> Ix( img.size()), Iy( img.size());
        cv::Sobel( img, Ix, CV_32F, 1, 0, 3);
        cv::Sobel( img, Iy, CV_32F, 0, 1, 3);

        RFeatures::showImage( img, "Original grey level (" + fname +")", false);
        FastHOG fasthog( nbins, knSz);
        const cv::Mat hogs = fasthog.makeHOGs( Ix, Iy);
        //const cv::Mat hogs = fasthog.makeHOGs( img);
        const cv::Mat_<float> anglesImg = fasthog.getMaxAngles();
        const cv::Mat_<float> magsImg = fasthog.getMaxMags();
        angleImgs.push_back(anglesImg);
        magImgs.push_back(magsImg);

        cv::Mat danglesImg, dmagsImg;
        anglesImg.convertTo( danglesImg, CV_8U, 255);
        magsImg.convertTo( dmagsImg, CV_8U, 255);
        RFeatures::showImage( danglesImg, "Angles (" + fname +")", false);
        RFeatures::showImage( dmagsImg, "Magnitudes (" + fname +")", true);
    }   // end for

    cv::Mat aimg;
    cv::Mat mimg;
    const int sz = angleImgs.size();
    for ( int i = 0; i < sz; ++i)
    {
        if ( aimg.empty())
            aimg = angleImgs[i] / sz;
        else
            aimg += angleImgs[i] / sz;

        if ( mimg.empty())
            mimg = magImgs[i] / sz;
        else
            mimg += magImgs[i] / sz;
    }   // end for
    cv::Mat daimg, dmimg;
    aimg.convertTo( daimg, CV_8U, 255);
    mimg.convertTo( dmimg, CV_8U, 255);
    RFeatures::showImage( daimg, "Mean angles", false);
    RFeatures::showImage( dmimg, "Mean magnitudes", true);

    return EXIT_SUCCESS;
}   // end main
