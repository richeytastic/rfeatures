#include <Superpixels.h>
#include <opencv2/opencv.hpp>
#include <string>
using std::string;
#include <vector>
#include <cstdlib>
#include <iostream>


void showImage( const cv::Mat &img, const string &title)
{
    cv::namedWindow( title.c_str());
    cv::imshow( title.c_str(), img);
}   // end showImage



cv::Mat createOverlayed( const cv::Mat &img, const cv::Mat &outlines)
{
    std::vector<cv::Mat> imgcs;
    cv::split( img, imgcs);
    imgcs[0] |= outlines;
    imgcs[1] |= outlines;
    imgcs[2] |= outlines;
    cv::Mat overlayed;
    cv::merge( imgcs, overlayed);
    return overlayed;
}   // end createOverlayed



int main( int argc, char **argv)
{
    using std::cerr;
    using std::endl;

    if ( argc != 4)
    {
        cerr << "Usage: " << argv[0] << " image_file_name num_superpixels compactness" << endl;
        return EXIT_FAILURE;
    }   // end if

    cv::Mat img = cv::imread( argv[1], true);
    if ( img.empty())
    {
        cerr << "Unable to load image from file " << argv[1] << endl;
        return EXIT_FAILURE;
    }   // end if

    const int numSPs = strtol( argv[2], NULL, 10);
    const int compactness = strtol( argv[3], NULL, 10);
    
    RFeatures::Superpixels spxls( img, numSPs, RFeatures::Superpixels::SP_COUNT, compactness);
    cv::Mat labels = spxls.createLabelImage();
    cv::Mat outlines = spxls.drawOutlines();
    cv::Mat overlayed = createOverlayed( img, outlines);

    showImage( img, "Original image");
    showImage( outlines, "Superpixel outlines");
    showImage( overlayed, "Overlayed");
    showImage( labels, "Superpixel labels");
    cv::waitKey();

    return EXIT_SUCCESS;
}   // end main
