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

#ifndef RFEATURES_CIRCLE_Diff_H
#define RFEATURES_CIRCLE_Diff_H

#include "FeatureOperator.h"

namespace RFeatures {

class rFeatures_EXPORT CircleDiff : public RFeatures::FeatureOperator
{
public:
    // numPoints: More points per circle gives more info but is longer to compute.
    CircleDiff( const cv::Mat_<float>& img, int numPoints);
    virtual ~CircleDiff();

    // Show the pattern of this feature (debug)
    //cv::Mat_<byte> showPattern( const cv::Size& imgSz) const;
    // Visualise the given feature vector as an image patch with sz dimensions.
    //cv::Mat_<byte> visFeature( const cv::Mat_<float>& fv, const cv::Size& imgSz) const;

protected:
    // Creates feature vectors, each having 3*numPoints elements
    // ranging from 0 to 1 and returns feature vector.
    virtual cv::Mat_<float> extract( const cv::Rect&) const;

private:
    const cv::Mat_<float> _img;
    const int _numPoints;

    vector<cv::Point2d> _points[3];
    struct ImageCircleDiffs;
    ImageCircleDiffs* _icds;
};  // end class

}   // end namespace

#endif
