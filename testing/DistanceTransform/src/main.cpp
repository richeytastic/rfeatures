//#define NDEBUG
#include <DistanceTransform.h>
using RFeatures::DistanceTransform;
#include <FeatureUtils.h>
#include <string>
using std::string;

#include <cstdlib>
#include <iostream>
using std::cerr;
using std::endl;


cv::Mat_<float> loadImageAndThreshold( const string& fname, int thresh)
{
    cv::Mat img = cv::imread( fname, false);   // Grey scale
    if ( img.channels() == 3)
    {
        cv::Mat greyImg;
        cv::cvtColor( img, greyImg, CV_BGR2GRAY);
        img = greyImg;
    }   // end if
    cv::Mat_<float> fimg;
    img.convertTo( fimg, CV_32F);
    cv::Mat_<float> timg;
    cv::threshold( fimg, timg, thresh, 1E20, cv::THRESH_BINARY_INV);
    return timg;
}   // end loadImageAndThreshold


void rshow( cv::Mat img, const string& imgTitle)
{
    const float sf = 512./std::min<int>(img.rows,img.cols); // Make min dimension 512 pixels
    cv::Mat dm;
    // resize without pixel interpolation
    cv::resize( RFeatures::convertForDisplay( img, true), dm, cv::Size(0,0), sf, sf, cv::INTER_NEAREST);
    RFeatures::showImage( dm, imgTitle);
}   // end rshow


int main( int argc, char** argv)
{
    if ( argc < 2)
    {
        cerr << "Provide image filename" << endl;
        return EXIT_FAILURE;
    }   // end if

    int thresh = 0;
    if ( argc > 2)
        thresh = strtof( argv[2], 0);

    const cv::Mat_<float> timg = loadImageAndThreshold( argv[1], thresh);

    DistanceTransform dt( timg);
    cv::Mat_<float> dtmap1 = dt();   // My version
    cv::Mat_<float> dtmap2 = dt.calcDefault();   // PFF's version

    // Compare differences
    const cv::Mat_<float> diffmap = dtmap2 - dtmap1;
    cv::Mat_<float> sumSqDiffs( diffmap.size());
    cv::pow( diffmap, 2, sumSqDiffs);
    const int nonzero = cv::countNonZero(sumSqDiffs);

    rshow( timg, "Original Threshold Image");
    cv::sqrt( dtmap1, dtmap1); // Square root to get real distances
    rshow(dtmap1, "Distance Transform (RP)");
    cv::sqrt( dtmap2, dtmap2); // Square root to get real distances
    rshow(dtmap2, "Distance Transform (PFF)");

    const bool writeImages = false;
    if ( nonzero > 0)
    {
        cerr << "Errors in DT calculation!" << endl;
        cerr << "Proportion of different pixels: " << (100 * double(nonzero)/(diffmap.total())) << "%" << endl;
        const cv::Mat_<float> v = sumSqDiffs > 0;
        rshow( v, "Squared differences");
        if ( writeImages)
        {
            cv::imwrite( "RP_DT.png", dtmap1);
            cv::imwrite( "PFF_DT.png", dtmap2);
            cv::imwrite( "Differences.png", v);
        }   // end if

        // Find and display the column having the most number of errors in it
        sumSqDiffs = sumSqDiffs.t();
        int maxErrRow = 0;
        int maxErrCnt = 0;
        for ( int i = 0; i < sumSqDiffs.rows; ++i)
        {
            int errCnt = 0;
            for ( int j = 0; j < sumSqDiffs.cols; ++j)
                if ( sumSqDiffs.at<float>( i, j) > 0)
                    errCnt++;
            if (errCnt > maxErrCnt)
            {
                maxErrRow = i;
                maxErrCnt = errCnt;
            }   // end if
        }   // end for - rows as columns

        cerr << "Largest number of errors in column " << maxErrRow << endl;
    }   // end if
    else
        cerr << "EXACT DT COMPUTED!" << endl;

    cv::waitKey();
    return EXIT_SUCCESS;
}   // end main

