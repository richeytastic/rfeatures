#include <FastHOGExtractor.h>
using RFeatures::FastHOGExtractor;
#include <PatchDescriptor.h>
using RFeatures::PatchDescriptor;
#include <FeatureUtils.h>
#include <iostream>
using std::cerr;
using std::endl;
#include <cstdlib>
typedef unsigned char byte;



int main( int argc, char** argv)
{
    if ( argc < 2)
    {
        cerr << "Provide image filename" << endl;
        return EXIT_FAILURE;
    }   // end if

    const string fname = argv[1];
    cv::Mat img = cv::imread( fname, true); // 3-channel
    /*
    if ( img.channels() == 3)
    {
        cerr << "Converting from colour to grey" << endl;
        cv::cvtColor( img, img, CV_BGR2GRAY);
    }   // end if
    */
    RFeatures::print(cerr, img);

    const cv::Size fvSize = img.size();
    const int nbins = 9;
    const cv::Size knSz(5,5);
    FastHOGExtractor fasthog( img.channels() == 3 ? true : false, fvSize, nbins, knSz);
    fasthog.processImage( img);

    PatchDescriptor::Ptr pd = PatchDescriptor::create();
    const int numFVs = fasthog.extract( cv::Rect(0,0,img.cols,img.rows), pd);

    cv::Mat dimg;
    for ( int i = 0; i < numFVs; ++i)
    {
        const cv::Mat_<float> fv = pd->getFeatureVector(i).reshape(0, fvSize.height) * 255;
        fv.convertTo( dimg, CV_8U);
        std::ostringstream oss;
        oss << "HOG" << i;
        RFeatures::showImage( dimg, oss.str(), (i < numFVs - 1) ? false : true);
    }   // end for

    return EXIT_SUCCESS;
}   // end main
