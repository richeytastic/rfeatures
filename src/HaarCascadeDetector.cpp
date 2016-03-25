#include "HaarCascadeDetector.h"
using RFeatures::HaarCascadeDetector;
#include <iostream>


HaarCascadeDetector::Ptr HaarCascadeDetector::create( const std::string& modelFile)
{
    HaarCascadeDetector::Ptr fd( new HaarCascadeDetector);
    if ( !fd->_classifier.load(modelFile))
        return HaarCascadeDetector::Ptr();
    return fd;
}   // end create


HaarCascadeDetector::Ptr HaarCascadeDetector::create( const HaarCascadeDetector::Ptr hcd)
{
    HaarCascadeDetector* hcd2 = new HaarCascadeDetector( *hcd);
    return HaarCascadeDetector::Ptr( hcd2);
}   // end create


void HaarCascadeDetector::setImage( const cv::Mat_<byte> img)
{
    _testImg = img;
}   // end setImage


size_t HaarCascadeDetector::detect( std::vector<cv::Rect>& bboxs) const
{
    if ( _testImg.empty())
    {
        std::cerr << "[ERROR] HaarCascadeDetector::detect(): No test image set!" << std::endl;
        return 0;
    }   // end if
    const size_t preSz = bboxs.size();
    _classifier.detectMultiScale( _testImg, bboxs);
    return bboxs.size() - preSz;
}   // end detect


HaarCascadeDetector::HaarCascadeDetector()
{}   // end ctor


HaarCascadeDetector::HaarCascadeDetector( const HaarCascadeDetector& hcd)
{
    _classifier = hcd._classifier;
    _testImg = hcd._testImg;
}   // end ctor
