/************************************************************************
 * Copyright (C) 2017 Richard Palmer
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 ************************************************************************/

#ifndef RFEATURES_HAAR_CASCADE_DETECTOR_H
#define RFEATURES_HAAR_CASCADE_DETECTOR_H

#ifdef _WIN32
#pragma warning( disable : 4251)
#endif

#include "rFeatures_Export.h"
#include <opencv2/opencv.hpp>
#include <boost/shared_ptr.hpp>
#include <vector>
#include <string>
typedef unsigned char byte;

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
