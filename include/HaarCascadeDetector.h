#pragma once
#ifndef RFEATURES_HAAR_CASCADE_DETECTOR_H
#define RFEATURES_HAAR_CASCADE_DETECTOR_H

#include "rFeatures_Export.h"
#include <opencv2/opencv.hpp>
typedef unsigned char byte;
#include <boost/shared_ptr.hpp>
#include <vector>
#include <string>

namespace RFeatures
{

class rFeatures_EXPORT HaarCascadeDetector
{
public:
    typedef boost::shared_ptr<HaarCascadeDetector> Ptr;

    // Use create( modelFile) to load the detector using the model definitions file.
    // Subsequent detectors can then be created using copies of this template using
    // the create( Ptr) static constructor.
    static Ptr create( const std::string& modelFile);
    static Ptr create( Ptr);

    // Image to detect (CV_8UC1)
    void setImage( const cv::Mat_<byte> img);

    // Detect instances of the object of interest, setting bounding boxes for their
    // occurence in img. Returns the number of detections (number of cv::Rects appended).
    size_t detect( std::vector<cv::Rect>& bboxs) const;

private:
    mutable cv::CascadeClassifier _classifier;
    cv::Mat_<byte> _testImg;

    HaarCascadeDetector();
    HaarCascadeDetector( const HaarCascadeDetector&);
    void operator=( const HaarCascadeDetector&);
};  // end class

}   // end namespace

#endif
