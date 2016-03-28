#include <cstdlib>
#include <iostream>
using namespace std;

#include <CannyOperator.h>
#include <HoughLinesOperator.h>
#include <HoughCirclesOperator.h>
#include <ConnectedComponents.h>
#include <HarrisKeypoints.h>
#include <WatershedOperator.h>
using namespace RFeatures;

#include <Exceptions.h>
using namespace rlib;
#include <ImageProcess.h>
#include <Histogram1D.h>
using namespace rlib::ImageProcess;


cv::Mat createCrossSE( int dim)
{
    if ( dim % 2 == 0)
        throw InvalidImageException( "Structuring Element dimensions must be odd!");

    cv::Mat se( dim, dim, CV_8U, cv::Scalar(0));
    se.row( dim/2).setTo(1);    // Integer division
    se.col( dim/2).setTo(1);    // Integer division

    return se;
}   // end createCrossSE



// Does a double unsharp mask on the image and reduces the colour space.
cv::Mat getEdgeHighlightedImage( const cv::Mat &img)
{
    cv::Mat imgL1;
    cv::GaussianBlur( img, imgL1, cv::Size(5,5), 1.5);
    imgL1 = 1.7*img - imgL1;

    colourReduce( imgL1, imgL1, 4);
    imgL1 = Histogram1D::stretch( imgL1);

    cv::Mat imgL2;
    cv::GaussianBlur( imgL1, imgL2, cv::Size(5,5), 1.5);
    imgL2 = 1.5*imgL1 - imgL2;

    cv::Mat imgL3;
    cv::GaussianBlur( imgL2, imgL3, cv::Size(5,5), 1.5);
    imgL3 = 1.3*imgL2 - imgL3;

    cv::Mat imgL4 = 0.7*imgL1 - 0.43*imgL2 + 1.21*imgL3;
    return 0.9*imgL4 + 0.4*img;
}   // end getEdgeHighlightedImage



// Return binary image with foreground pixels as white
cv::Mat getForeground( const cv::Mat &image)
{
    cv::Mat img = getEdgeHighlightedImage( image);

    //cv::namedWindow("Canny Input Image");
    //cv::imshow("Canny Input Image", img);

    CannyOperator co( img, 170, 200);
    cv::Mat fgImg = co.getEdgeImage();

    //cv::namedWindow("Canny Output Image");
    //cv::imshow("Canny Output Image", fgImg);

    // Overlay with thicker lines 
    HoughLinesOperator hlo( co, 70, 20, 10);
    HoughLinesOperator::drawLines( hlo.findLines(), fgImg, cv::Scalar(255), 2);

    // Overlay with found circles
    cv::Mat imgC = image.clone();
    cv::GaussianBlur( imgC, imgC, cv::Size(3,3), 1.5);
    cv::medianBlur( imgC, imgC, 3);
    HoughCirclesOperator hco( imgC, 200, 35, 50, 7, 80);
    HoughCirclesOperator::drawCircles( hco.findCircles(), fgImg, cv::Scalar(255), 4);

    //cv::namedWindow("Hough Adjusted Image");
    //cv::imshow("Hough Adjusted Image", fgImg);

    // Find and draw corners
    HarrisKeypoints corners( img, 100, 0.1, 10);
    corners.draw( corners.find(), &fgImg);

    // Dilate lines with an emphasis on horizontal and vertical lines (diamond structuring element)
    cv::Mat diamondSE( 5, 5, CV_8U, cv::Scalar(1));

    diamondSE.row(0).setTo(0);
    diamondSE.row(4).setTo(0);
    diamondSE.col(0).setTo(0);
    diamondSE.col(4).setTo(0);
    diamondSE.at<uchar>(0,2) = 1;
    diamondSE.at<uchar>(4,2) = 1;
    diamondSE.at<uchar>(2,0) = 1;
    diamondSE.at<uchar>(2,4) = 1;

    cv::morphologyEx( fgImg, fgImg, cv::MORPH_CLOSE, diamondSE);

    //cv::namedWindow("Morphologically Adjusted Image");
    //cv::imshow("Morphologically Adjusted Image", fgImg);

    // Derive connected components and filter those that are too small.
    ContoursVector cvs = ConnectedComponents( fgImg, 22).findComponents();
    cv::Mat retImg( fgImg.size(), fgImg.type(), cv::Scalar(0)); // Blank image
    ConnectedComponents::drawContours( cvs, retImg, cv::Scalar(255), -1);
    cv::dilate( retImg, retImg, cv::Mat(), cv::Point(-1,-1), 3);

    return retImg;
}   // end getForeground



cv::Mat formMarkersImage( const cv::Mat &fgImg)
{
    int bgCol = 128;

    // Background is simply the inverse of a severely dilated foreground with some
    // contextual removal from top and bottom.
    cv::Mat bgImg;
    cv::dilate( fgImg, bgImg, cv::Mat(), cv::Point(-1,-1), 25);  // Dilate foreground a lot
    cv::threshold( bgImg, bgImg, 1, bgCol, cv::THRESH_BINARY_INV);    // Background set as grey (128)

    bgImg += fgImg; // add the foreground components

    // Contextual background given as bands at top and bottom of the image.
    // Within these areas, we do not expect to see any interesting objects.
    int topBgWidth = 90;    // Draw top band
    cv::rectangle( bgImg, cv::Rect(0,0, bgImg.cols, topBgWidth), cv::Scalar(bgCol), CV_FILLED);
    int botBgWidth = 60;    // Draw bottom band
    cv::rectangle( bgImg, cv::Rect(0, bgImg.rows - botBgWidth, bgImg.cols, botBgWidth), cv::Scalar(bgCol), CV_FILLED);

    return bgImg;
}   // end formMarkersImage



int main( int argc, char **argv)
{
    cv::Mat image = cv::imread(argv[1]); // Load as colour
    if ( image.empty())
    {
        cerr << "No image loaded!" << endl;
        return EXIT_FAILURE;
    }   // end if

    cv::Mat greyImg;    // Convert to single channel grey image
    cv::cvtColor( image, greyImg, CV_BGR2GRAY);

    // Show original image
    cv::namedWindow("Original");
    cv::imshow("Original", image);

    cv::Mat fgImg = getForeground( greyImg);
    cv::Mat markers = formMarkersImage( fgImg);

    // Show initial markers image
    cv::namedWindow("Markers");
    cv::imshow("Markers", markers);

    cv::Mat mimg;
    cv::medianBlur( image, mimg, 5);
    colourReduce( mimg, mimg, 32);
    mimg = Histogram1D::stretch( mimg);

    cv::Mat segImg = WatershedOperator( mimg, markers)();
    cv::threshold( segImg, segImg, 200, 255, cv::THRESH_BINARY);

    // Segment the original image
    cv::cvtColor( segImg, segImg, CV_GRAY2BGR);
    cv::Mat segmented = image & segImg;

    // Show weighted segmented image
    cv::namedWindow("Segmentation");
    cv::imshow("Segmentation", segmented);

    cv::waitKey();

    return EXIT_SUCCESS;
}   // end main
