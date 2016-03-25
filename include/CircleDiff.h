#pragma once
#ifndef RFEATURES_CIRCLE_Diff_H
#define RFEATURES_CIRCLE_Diff_H

#include "FeatureOperator.h"
//typedef unsigned char byte;


namespace RFeatures
{

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




