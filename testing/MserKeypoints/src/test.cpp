#include <cstdlib>
#include <iostream>
using namespace std;

#include <MserKeypoints.h>
using namespace RFeatures;



int main( int argc, char **argv)
{
    cv::Mat image = cv::imread(argv[1]);
    if ( image.empty())
    {
        cerr << "No image loaded!" << endl;
        return EXIT_FAILURE;
    }   // end if

    cv::Mat fdImg = MserKeypoints( image, 3, 1, 400, 250.0, 1.0, 250, 10.0, 1.0, 3)();
    cv::namedWindow("Mser Keypoints");
    cv::imshow("Mser Keypoints", fdImg);

    cv::waitKey();

    return EXIT_SUCCESS;
}   // end main
