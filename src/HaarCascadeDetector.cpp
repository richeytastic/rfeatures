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

#include <HaarCascadeDetector.h>
using RFeatures::HaarCascadeDetector;
#include <iostream>


HaarCascadeDetector::Ptr HaarCascadeDetector::create( const std::string& modelFile)
{
    Ptr fd( new HaarCascadeDetector);
    if ( !fd->_classifier.load(modelFile))
        return HaarCascadeDetector::Ptr();
    return fd;
}   // end create


void HaarCascadeDetector::setImage( const cv::Mat_<byte> img) { _testImg = img;}


size_t HaarCascadeDetector::detect( std::vector<cv::Rect>& bboxs) const
{
    if ( _testImg.empty())
    {
        std::cerr << "[ERROR] RFeatures::HaarCascadeDetector::detect(): No test image set!" << std::endl;
        return 0;
    }   // end if
    const size_t preSz = bboxs.size();
    _classifier.detectMultiScale( _testImg, bboxs);
    return bboxs.size() - preSz;
}   // end detect


/*
HaarCascadeDetector::HaarCascadeDetector( const HaarCascadeDetector& hcd)
{
    _classifier = hcd._classifier;
    _testImg = hcd._testImg;
}   // end ctor
*/
