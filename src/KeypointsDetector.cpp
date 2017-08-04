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

#include "KeypointsDetector.h"
using namespace RFeatures;


KeypointsDetector::KeypointsDetector( const cv::Mat &img)
{
    original_image = img.clone();
}   // end ctor


KeypointsDetector::~KeypointsDetector()
{
    original_image.release();
}   // end dtor


cv::Mat KeypointsDetector::draw( const Keypoints &kps, cv::Scalar col, cv::Mat *img) const
{
    bool useProvided( img != NULL);
    cv::Mat outImg;
    if ( !useProvided)
        outImg = original_image.clone();

    cv::drawKeypoints( original_image, kps,
            ( useProvided ? *img : outImg), col,
            cv::DrawMatchesFlags::DRAW_OVER_OUTIMG | keypointDrawFlag());
    return ( useProvided ? *img : outImg);
}   // end draw


cv::Mat KeypointsDetector::draw( const Keypoints &kps, cv::Mat *img, cv::Scalar col) const
{
    return draw( kps, col, img);
}   // end draw


cv::Mat KeypointsDetector::operator()() const
{
    return this->draw( this->find());
}   // end operator()


Keypoints KeypointsDetector::detectKeypoints( cv::FeatureDetector &fd, const cv::Mat &img) const
{
    Keypoints keypoints;    // vector<cv::Keypoint> --> Keypoints
    fd.detect( img, keypoints);
    return keypoints;
}   // end detectKeypoints


int KeypointsDetector::keypointDrawFlag() const
{
    return cv::DrawMatchesFlags::DEFAULT;   // 0
}   // end keypointDrawFlag


cv::Mat KeypointsDetector::cloneOriginalImage() const
{
    return original_image.clone();
}   // end cloneOriginalImage
