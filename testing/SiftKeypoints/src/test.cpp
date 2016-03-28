#include <cstdlib>
#include <iostream>
using namespace std;

#include <SiftKeypoints.h>
using namespace RFeatures;



int main( int argc, char **argv)
{
    cv::Mat image = cv::imread(argv[1]);
    if ( image.empty())
    {
        cerr << "No image loaded!" << endl;
        return EXIT_FAILURE;
    }   // end if

    cv::Mat fdImg = SiftKeypoints( image, 0.1, 10)();
    cv::namedWindow("Sift Keypoints");
    cv::imshow("Sift Keypoints", fdImg);

    cv::waitKey();

    return EXIT_SUCCESS;
}   // end main
