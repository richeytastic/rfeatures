#include "LinesConverter.h"
using RFeatures::LinesConverter;
using RFeatures::InvalidImageException;
#include <LinearRegressor.h>    // rlib
#include <boost/foreach.hpp>
#include <boost/math/special_functions/fpclassify.hpp>
#include <queue>


LinesConverter::LinesConverter( const Lines &lns, const cv::Mat &ri, float ml, float fl)
                                throw (InvalidImageException)
    : rngData(ri), lines(lns), minLen(ml), focLen(fl)
{
    if ( ri.type() != CV_32FC1)
        throw InvalidImageException( ri, "Invalid image type for LinesConverter::ctor!");
}   // end ctor



Lines3d LinesConverter::get3DLines() const
{
    Lines3d lns;  // New lines vector

    float xcentre = (float)(rngData.cols - 1)/2;
    float ycentre = (float)(rngData.rows - 1)/2;
    float x, y, xend, yend, xdiff, ydiff, xinc, yinc, len, lenInc;
    float xst, yst;

    BOOST_FOREACH( cv::Vec4i ln, lines)
    {
        x = (float)ln[0];
        y = (float)ln[1];
        xend = (float)ln[2];
        yend = (float)ln[3];

        ydiff = yend - y;
        xdiff = xend - x;
        len = sqrt(ydiff*ydiff+xdiff*xdiff);

        xinc = xdiff / len;
        yinc = ydiff / len;
        lenInc = sqrt(xinc*xinc+yinc*yinc);

        while ( len >= minLen)
        {
            // Skip no depth segment 
            while ( (rngData.at<float>((int)y,(int)x) <= 0) && len > 0.0)
            {
                y += yinc;
                x += xinc;
                len -= lenInc;
            }   // end while

            cv::Vec6f v3;    // New segment

            // Store start point for this segment
            v3[2] = rngData.at<float>( (int)y, (int)x);
            v3[0] = v3[2] * (xcentre - x) / focLen;
            v3[1] = v3[2] * (ycentre - y) / focLen;
            xst = x;
            yst = y;

            // Skip depth segment
            while ( rngData.at<float>((int)y,(int)x) > 0 && len > 0.0)
            {
                y += yinc;
                x += xinc;
                len -= lenInc;
            }   // end while

            // Ensure we have an end-point with valid depth for this line segment
            v3[5] = rngData.at<float>( (int)(y - yinc), (int)(x - xinc));
            v3[3] = v3[5] * (xcentre - (x - xinc)) / focLen;
            v3[4] = v3[5] * (ycentre - (y - yinc)) / focLen;

            // We only want lines that are long enough as measured in 2D...
            if ( sqrt( pow(xst-(x-xinc),2) + pow(yst-(y-yinc),2)) >= minLen)
                lns.push_back(v3);
        }   // end while
    }   // end foreach

    return lns;
}   // end get3DLines



struct FittedLine
{
    FittedLine( float sx, float sy, float sz, float ex, float ey, float ez, double r2)
    {
        rsq = r2;
        v[0] = sx;
        v[1] = sy;
        v[2] = sz;
        v[3] = ex;
        v[4] = ey;
        v[5] = ez;
    }   // end ctor

    bool operator()( const FittedLine &lhs, const FittedLine &rhs) const
    {
        return lhs.rsq < rhs.rsq;
    }   // end operator()


    bool operator<( const FittedLine &fl) const
    {
        return rsq < fl.rsq;
    }   // end operator<

    double rsq;
    cv::Vec6f v; // 3D line segment
};  // end class FittedLine



Lines3d LinesConverter::fitAndFilter( double minRSquared) const
{
    std::priority_queue<FittedLine> pqLines; // Priority queue for filtering

    float xcentre = (float)(rngData.cols - 1)/2;
    float ycentre = (float)(rngData.rows - 1)/2;
    float x, y, xend, yend, xdiff, ydiff, xinc, yinc, len, lenInc;
    float xst, yst, d, startVal, endVal;
    float sx,sy,sz,ex,ey,ez;
    double r2;

    double scale = -255.0 / 30.0;

    int segCount = 0;
    int shortSegs = 0;
    int usedSegs = 0;

    BOOST_FOREACH( cv::Vec4i ln, lines)
    {
        x = (float)ln[0];
        y = (float)ln[1];
        xend = (float)ln[2];
        yend = (float)ln[3];

        ydiff = yend - y;
        xdiff = xend - x;
        len = sqrt(ydiff*ydiff+xdiff*xdiff);

        xinc = xdiff / len;
        yinc = ydiff / len;
        lenInc = sqrt(xinc*xinc+yinc*yinc);

        while ( len >= minLen)
        {
            // Skip no depth segment 
            while ( rngData.at<float>((int)y,(int)x) <= 0 && len > 0.0)
            {
                y += yinc;
                x += xinc;
                len -= lenInc;
            }   // end while

            std::vector<float> depthVals;    // Scaled depth values along this segment

            //depthVals.push_back( rngData.at<float>( (int)y, (int)x));
            d = cv::saturate_cast<uchar>(scale * rngData.at<float>( (int)y, (int)x) + 255); // Scale depth
            depthVals.push_back( d);

            xst = x;
            yst = y;

            // Skip depth segment
            while ( (d = rngData.at<float>((int)y,(int)x)) > 0 && len > 0.0)
            {
                y += yinc;
                x += xinc;
                len -= lenInc;
                //depthVals.push_back(d);
                depthVals.push_back( cv::saturate_cast<uchar>(scale * d + 255));  // Scale depth
            }   // end while

            // We only want lines that are long enough as measured in 2D...
            if ( sqrt( pow(xst-(x-xinc),2) + pow(yst-(y-yinc),2)) >= minLen)
            {
                rlib::LinearRegressor lr( depthVals);
                r2 = lr.calcSmoothedLine();
                if ( r2 < minRSquared)  // Skip this line segment if too poor a fit
                    continue;

                usedSegs++;

                startVal = float((lr.getStartPoint() - 255) / scale);
                endVal = float((lr.getEndPoint() - 255) / scale);

                // Start point for this segment
                sz = startVal;
                sx = startVal * (xcentre - xst) / focLen;
                sy = startVal * (ycentre - yst) / focLen;

                // End point for this segment
                ez = endVal;
                ex = endVal * (xcentre - (x - xinc)) / focLen;
                ey = endVal * (ycentre - (y - yinc)) / focLen;

                // Now we order according to the R^2 value
                FittedLine fl( sx, sy, sz, ex, ey, ez, r2);
                pqLines.push(fl);
            }   // end if
            else
            {
                shortSegs++;
            }   // end else

            segCount++;
        }   // end while
    }   // end foreach

    Lines3d sortedLines;  // New lines vector
    while ( !pqLines.empty())
    {
        sortedLines.push_back( pqLines.top().v);
        pqLines.pop();
    }   // end while

    return sortedLines;
}   // end fitAndFilter
