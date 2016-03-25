#include "SobelMaker.h"
using RFeatures::SobelMaker;
#include <cassert>
#include <iostream>


SobelMaker::SobelMaker( const cv::Mat& img)
{
    assert( img.channels() == 1);
    double mn, mx;
    cv::minMaxLoc( img, &mn, &mx);
    const cv::Mat timg = img - mn;
    timg.convertTo( _img, CV_32F, 1./(mx-mn));
}   // end ctor



cv::Mat_<float> SobelMaker::makeSobelD1( int ksize) const
{
    const cv::Size imgSz = _img.size();
    cv::Mat_<float> dst0( imgSz);
    cv::Mat_<float> dst1( imgSz);
    cv::Sobel( _img, dst0, -1, 1, 0, ksize, 1, 0);
    cv::Sobel( _img, dst1, -1, 0, 1, ksize, 1, 0);
    dst0 = cv::abs(dst0);
    dst1 = cv::abs(dst1);

    cv::Mat_<float> dst = dst0 + dst1;
    double mn, mx;
    cv::minMaxLoc( dst, &mn, &mx);
    //std::cerr << "Sobel D1 mn, mx = " << mn << ", " << mx << std::endl;
    dst *= 1./mx;
    return dst;
}   // end makeSobelD1



cv::Mat_<float> SobelMaker::makeSobelD2( int ksize) const
{
    const cv::Size imgSz = _img.size();
    cv::Mat_<float> dst0( imgSz);
    cv::Mat_<float> dst1( imgSz);
    cv::Sobel( _img, dst0, -1, 2, 0, ksize, 1, 0);
    cv::Sobel( _img, dst1, -1, 0, 2, ksize, 1, 0);
    dst0 = cv::abs(dst0);
    dst1 = cv::abs(dst1);

    cv::Mat_<float> dst = dst0 + dst1;
    double mn, mx;
    cv::minMaxLoc( dst, &mn, &mx);
    dst *= 1./mx;
    return dst;
}   // end makeSobelD2

