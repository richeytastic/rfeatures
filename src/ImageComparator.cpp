#include <ImageComparator.h>
using namespace RFeatures;
#include <vector>


ImageComparator::ImageComparator( int factor, const cv::Mat &refImg)
{
    setColourReduction( factor);
    setReferenceImage( refImg);
}   // end ctor



double ImageComparator::operator()( const cv::Mat &img)
{
    // Resize image to be the same as the reference image
    cv::Mat image = img;
    if ( image.size() != imgsz) // Ensure size is same as reference image
        resize( image, image, imgsz);
    inputH = hist.getHistogram(image);
    double v = cv::compareHist( refH, inputH, CV_COMP_INTERSECT) + 1;
    return v / ((imgsz.width * imgsz.height) + 2);
}   // end operator()



void ImageComparator::setColourReduction( int factor)
{
    if ( factor < 1)
        factor = 1;
    div = factor;
}   // end seColourReduction



void ImageComparator::setReferenceImage( const cv::Mat &img)
{
    cv::Mat ref;
    colourReduce( img, ref, div);
    imgsz = ref.size();
    refH = hist.getHistogram( ref);
}   // end setReferenceImage
