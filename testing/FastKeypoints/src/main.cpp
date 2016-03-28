#include <cstdlib>
#include <iostream>
using namespace std;

#include <FastKeypoints.h>
using namespace RFeatures;



int main( int argc, char **argv)
{
    if ( argc != 2)
    {
        cerr << "Provide an image file" << endl;
        return EXIT_FAILURE;
    }   // end if

    cv::Mat image = cv::imread(argv[1]);
    if ( image.empty())
    {
        cerr << "No image loaded!" << endl;
        return EXIT_FAILURE;
    }   // end if

    cv::namedWindow("Original");
    cv::imshow("Original", image);

    FastKeypoints fk( image, 10);
    const Keypoints kps = fk.find();
    cout << "Got " << kps.size() << " keypoints" << endl;

    cv::Mat fdImg = image.clone();
    fk.draw( kps, &fdImg);
    cv::namedWindow("Fast Keypoints");
    cv::imshow("Fast Keypoints", fdImg);

    cv::waitKey();

    return EXIT_SUCCESS;
}   // end main
