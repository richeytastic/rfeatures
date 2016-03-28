#include <ImageLabeller.h>
#include <vector>
#include <cstdlib>
#include <iostream>


int main( int argc, char** argv)
{
    if ( argc != 2)
    {
        std::cerr << "Usage: " << argv[0] << " binary_img_filename" << std::endl;
        return EXIT_FAILURE;
    }   // end if

    const cv::Mat img = cv::imread( std::string(argv[1]), false);
    if ( img.empty())
    {
        std::cerr << "Unable to load image!" << std::endl;
        return EXIT_FAILURE;
    }   // end if

    RFeatures::ImageLabeller ilabeller( img, 255);
    const int numRegs = ilabeller();
    std::vector<int> regSizes;
    ilabeller.getRegionSizes( regSizes);

    std::cerr << "Got " << numRegs << " FG regions" << std::endl;
    for ( int i = 0; i < numRegs; ++i)
        std::cerr << "FG Obj " << i << " num pixels = " << regSizes[i] << std::endl;

    cv::namedWindow( "Original image");
    cv::imshow( "Original image", img);
    cv::namedWindow( "Label image");
    const cv::Mat_<byte> labImg = ilabeller.getLabelImage();
    cv::imshow( "Label image", labImg);
    cv::waitKey();

    cv::imwrite( "labImg.png", labImg);

    return EXIT_SUCCESS;
}   // end main
