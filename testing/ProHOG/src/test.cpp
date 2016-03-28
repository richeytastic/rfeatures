#include <cstdlib>
#include <cmath>
#include <iostream>
using namespace std;
#include <cassert>
#include <sys/time.h>

#include "ProHOG.h"
#include "FeatureUtils.h"
using namespace RFeatures;


int main( int argc, char **argv)
{
    if ( argc < 4)
    {
        cerr << "Please provided image name and cell dimensions (width then height)." << endl;
        return EXIT_FAILURE;
    }   // end if

    cv::Mat image;
    if ( !loadImage( argv[1], image))
    {
        cerr << "Could not load image " << argv[1] << endl;
        return EXIT_FAILURE;
    }   // end if

    print( cout, image);
    showImage( image, "Original", false);

    const int nbins = 8;
    bool contrastVariant = true;
    bool spatialSmooth = true;
    bool gammaCorrect = true;
    // Radians per histogram bin
    double binRads = 1./nbins * (contrastVariant ? 2 : 1) * M_PI;

    const int cellsWide = (int)strtol( argv[2], NULL, 10);
    const int cellsHigh = (int)strtol( argv[3], NULL, 10);

    cout << "Calculating Pro-HOG map of size (width,height): " << cellsWide << ", " << cellsHigh << endl;
    cout << "Press spacebar to calculate..." << endl;
    while ( (int)(char)cv::waitKey() != 32);   // Wait for space-bar

    struct timeval ts;
    gettimeofday( &ts, NULL);

    /*
    ImageGradientsBuilder::Ptr gradsBuilder
        = ImageGradientsBuilder::create( image, nbins, contrastVariant, spatialSmooth, gammaCorrect);
    IntegralImage<double>::Ptr pxlGrads = gradsBuilder->getIntegralGradients();

    ProHOG::Ptr phog = ProHOG::create( pxlGrads);
    */
    ProHOG phog( image);

    // Set which part of image we want to extract
    //cv::Rect extract( image.cols/4, image.rows/4, image.cols/2, image.rows/2);  // Middle part of image
    cv::Rect extract( 0, 0, image.cols, image.rows);  // Whole image

    cv::Mat phogs = phog( cv::Size( cellsWide, cellsHigh), extract);
    struct timeval te;
    gettimeofday( &te, NULL);
    double usecs = 1000000 * (te.tv_sec - ts.tv_sec) + (te.tv_usec - ts.tv_usec);
    cerr << "Calculated Pro-HOG map in " << usecs / 1000 << " msecs" << endl;

    // Visualise at same size as original image
    cv::Mat pimg = ProHOG::createVisualisation( phogs, cv::Size( extract.width, extract.height), binRads);
    showImage( pimg, "Pro-HOG visualisation");
    if ( saveImage( "phog_img.png", pimg))
        cout << "Saved" << endl;
    else
        cout << "Not saved!" << endl;

    cout << "Press space to exit..." << endl;
    int k = 0;
    while ( (k = (int)(char)cv::waitKey()) != 32 && k != 27);
    return EXIT_SUCCESS;
}   // end main



