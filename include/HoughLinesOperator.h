#pragma once
#ifndef RFEATURES_HOUGH_LINES_OPERATOR_H
#define RFEATURES_HOUGH_LINES_OPERATOR_H

#include "CannyOperator.h"
using RFeatures::CannyOperator;
#include "RFeatures.h"
#ifndef M_PI
#define M_PI 3.14159265359
#endif

namespace RFeatures
{
class rFeatures_EXPORT HoughLinesOperator
{
public:
    HoughLinesOperator( const CannyOperator &canny, int minVote=20, int minLength=10, int maxGap=0,
                        int deltaRho=1, double deltaTheta=M_PI/360);

    virtual ~HoughLinesOperator();

    // Configuration parameters
    void setDetectionParams( int deltaRho, double deltaTheta, int minVotes);
    void setLineParams( int minLength, int maxGap);

    // Detect line segments where the components of the vector are the X,Y coords
    // for the two end points of the line segments.
    void findLines( Lines &lns) const;

    // Draw given lines on given image in given colour (default white) with given thickness (default 1 pxl)
    static void drawLines( const Lines &lines, cv::Mat &img,
            cv::Scalar colour=cv::Scalar(255,255,255), int thick=1);

private:
    CannyOperator canny;

    int deltaRho;
    double deltaTheta;

    int minVote;     // Min number of votes a line must receive before being considered
    int minLength;   // Min length for a line
    int maxGap;      // Max allowed gap along a line
};  // end class HoughLinesOperator

}   // end namespace RFeatures

#endif
