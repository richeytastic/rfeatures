#include <GallLempitskyFeatureExtractor.h>
#include <FeatureUtils.h>
#include <cassert>
#include <string>
using std::string;
#include <iostream>
using std::cout;
using std::cerr;
using std::endl;
#include <cstdlib>
typedef unsigned char byte;


int main( int argc, char** argv)
{
    if ( argc < 2)
    {
        cerr << "Provide image filename [-g to load as grey scale]" << endl;
        return EXIT_FAILURE;
    }   // end if

    bool loadColour = true;
    if ( argc > 2 && string(argv[1]) == "-g")
        loadColour = false;

    const cv::Mat img = cv::imread( argv[argc-1], loadColour);
    RFeatures::print( cout, img);

    const cv::Size gfPatchSz = img.size();
    //const cv::Size gfPatchSz( 16, 16);  // Nominal feature scaling size
    RFeatures::GallLempitskyFeatureExtractor glf( img.channels() == 3 ? false : true, gfPatchSz);
    glf.processImage( img);

    vector<cv::Mat_<float> > fvs;
    const int numFVs = glf.extract( cv::Rect(0,0,img.cols,img.rows), fvs);
    cerr << "Got " << numFVs << " feature vectors" << endl;

    vector<cv::Mat> dimgs( fvs.size());
    for ( int i = 0; i < fvs.size(); ++i)
    {
        double mn, mx;
        cv::minMaxLoc( fvs[i], &mn, &mx);
        cerr << "Feature " << i << " min, max = " << mn << ", " << mx << endl;
        cv::Mat outImg;
        fvs[i].convertTo( outImg, CV_8U);
        dimgs[i] = outImg.reshape(0, gfPatchSz.height);
    }   // end for

    RFeatures::showImage( dimgs[0], "CIE Lab - L", false);
    RFeatures::showImage( dimgs[1], "Sobel X", false);
    RFeatures::showImage( dimgs[2], "Sobel Y", false);
    RFeatures::showImage( dimgs[3], "Sobel XX", false);
    RFeatures::showImage( dimgs[4], "Sobel YY", false);
    RFeatures::showImage( dimgs[5], "HOG bin 0", false);
    RFeatures::showImage( dimgs[6], "HOG bin 1", false);
    RFeatures::showImage( dimgs[7], "HOG bin 2", false);
    RFeatures::showImage( dimgs[8], "HOG bin 3", false);
    RFeatures::showImage( dimgs[9], "HOG bin 4", false);
    RFeatures::showImage( dimgs[10], "HOG bin 5", false);
    RFeatures::showImage( dimgs[11], "HOG bin 6", false);
    RFeatures::showImage( dimgs[12], "HOG bin 7", false);
    RFeatures::showImage( dimgs[13], "HOG bin 8", false);
    RFeatures::showImage( dimgs[14], "HOG max angles", false);
    RFeatures::showImage( dimgs[15], "HOG max values", true);

    return EXIT_SUCCESS;
}   // end main
