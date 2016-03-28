#include <cstdlib>
#include <iostream>
using namespace std;
#include <IntegralImage.h>
#include <FeatureUtils.h>
using namespace RFeatures;

#include <cassert>


int main( int argc, char **argv)
{
    cv::Mat image;
    if (!loadImage( argv[1], image))
    {
        cerr << "Couldn't load image!" << endl;
        return EXIT_FAILURE;
    }   // end if

    IntegralImage<uchar> intImg( image);

    cv::Rect rct(0,0,1,1);
    for ( int i = 1; i < image.rows; ++i)
    {
        for ( int j = 1; j < image.cols; ++j)
        {
            rct.y = i;
            rct.x = j;
            const cv::Vec3b &pxl = image.at<cv::Vec3b>(i,j);
            double b1 = (double)pxl[0];
            double b2 = intImg( rct, 0);
            double g1 = (double)pxl[1];
            double g2 = intImg( rct, 1);
            double r1 = (double)pxl[2];
            double r2 = intImg( rct, 2);
            assert( b1 == b2 && g1 == g2 && r1 == r2);    // Yes, these are doubles being compared exactly!
        }   // end for
    }   // end for

    showImage( image, "Original");
    showImage( convertForDisplay( intImg.getImage()), "Integral", true);

    return EXIT_SUCCESS;
}   // end main



