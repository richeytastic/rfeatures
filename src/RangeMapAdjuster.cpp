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

#include "RangeMapAdjuster.h"
using RFeatures::RangeMapAdjuster;
#include <algorithm>
#include <boost/unordered_map.hpp>


RangeMapAdjuster::RangeMapAdjuster( const cv::Mat_<float> &rngMap, float depthMax)
    : rngMap_(rngMap), depthMax_(depthMax)
{
}   // end ctor


// public
cv::Mat_<float> RangeMapAdjuster::operator()( const cv::Mat_<cv::Vec3b> &img) const
{
    const cv::Mat_<float> rngMap = rngMap_;
    assert( img.size() == rngMap.size());
    Superpixels segment( img, 170, Superpixels::SP_PIXELS, 3);
    int numLabels = 0;
    const int* const labs = segment.extract( numLabels);

    const int rows = img.rows;
    const int cols = img.cols;

    // Collect the minimum range values from each of the superpixels
    boost::unordered_map<int, float> labelRanges;
    int k = 0;  // index into labs
    for ( int i = 0; i < rows; ++i)
    {
        const float *rngRow = rngMap.ptr<float>(i);
        for ( int j = 0; j < cols; ++j)
        {
            const int label = labs[k++];    // label for this pixel
            float rng = rngRow[j];
            if ( rng == 0)
                rng = depthMax_;
            if ( labelRanges.count(label) == 0 || (rng < labelRanges.at(label)))
                labelRanges[label] = rng;
        }   // end for - cols
    }   // end for - rows

    cv::Mat_<float> outRngMap( rngMap.size());

    k = 0;
    for ( int i = 0; i < rows; ++i)
    {
        float *outRng = outRngMap.ptr<float>(i);
        for ( int j = 0; j < cols; ++j)
            outRng[j] = labelRanges.at(labs[k++]);
    }   // end for - rows

    return outRngMap;
}   // end operator()



// public
cv::Mat_<float> RangeMapAdjuster::operator()( const cv::Size2f &modSz) const
{
    const cv::Mat_<float> rngMap = rngMap_;

    const cv::Mat_<float> hRngMap = adjustRangeMapHeight( rngMap, modSz);
    cv::Mat_<float> lRngMap = cv::Mat_<float>( rngMap.size());
    cv::Mat_<float> rRngMap = cv::Mat_<float>( rngMap.size());

    const int rows = rngMap.rows;
    const int halfRows = rows/2;
    const int cols = rngMap.cols;
    const int halfCols = cols/2;

    float* curWLRng = (float*)cv::fastMalloc(rows * sizeof(float));
    int* pxlWLCount = (int*)cv::fastMalloc(rows * sizeof(int));
    float* curWRRng = (float*)cv::fastMalloc(rows * sizeof(float));
    int* pxlWRCount = (int*)cv::fastMalloc(rows * sizeof(int));

    for ( int i = 0; i < cols; ++i)
    {
        curWLRng[i] = depthMax_;
        pxlWLCount[i] = 0;
        curWRRng[i] = depthMax_;
        pxlWRCount[i] = 0;
    }   // end for

    for ( int i = 0; i < cols; ++i)
    {
        const int k = cols-i;

        for ( int j = 0; j < rows; ++j)
        {
            // Runs from left to right
            float depth = std::min( depthMax_, hRngMap.at<float>(j,i));
            if ( depth == 0)
                depth = depthMax_;

            if ( depth < curWLRng[j])
            {
                curWLRng[j] = depth;
                pxlWLCount[j] = (int)(modSz.width/depth * halfCols + 0.5);
            }   // end if

            pxlWLCount[j]--;
            if (pxlWLCount[j] <= 0)
                curWLRng[j] = depth; // Reset
            lRngMap.at<float>(j,i) = curWLRng[j];

            // Runs from right to left
            depth = std::min( depthMax_, hRngMap.at<float>(j,k));  // Row j, column k
            if ( depth == 0)
                depth = depthMax_;

            if ( depth < curWRRng[j])
            {
                curWRRng[j] = depth;
                pxlWRCount[j] = (int)(modSz.width/depth * halfCols + 0.5);
            }   // end if

            pxlWRCount[j]--;
            if (pxlWRCount[j] <= 0)
                curWRRng[j] = depth; // Reset
            rRngMap.at<float>(j,k) = curWRRng[j];
        }   // end for - rows
    }   // end for - cols

    /*
    const cv::Mat dRngMap = RFeatures::convertForDisplay( rngMap, true);
    RFeatures::showImage( dRngMap, "Range map (original)", false);
    const cv::Mat dhRngMap = RFeatures::convertForDisplay( hRngMap, true);
    RFeatures::showImage( dhRngMap, "Height range map", false);
    const cv::Mat dlRngMap = RFeatures::convertForDisplay( lRngMap, true);
    RFeatures::showImage( dlRngMap, "Left range map", false);
    const cv::Mat drRngMap = RFeatures::convertForDisplay( rRngMap, true);
    RFeatures::showImage( drRngMap, "Right range map", false);
    */

    cv::Mat_<float> mRngMap = cv::Mat_<float>( rngMap.size());
    for ( int i = 0; i < rows; ++i)
        for ( int j = 0; j < cols; ++j)
            mRngMap.at<float>(i,j) = std::min( lRngMap.at<float>(i,j), rRngMap.at<float>(i,j));

    //RFeatures::showImage( RFeatures::convertForDisplay( mRngMap, true), "Min range map", false);

    cv::fastFree(curWLRng);
    cv::fastFree(pxlWLCount);
    cv::fastFree(curWRRng);
    cv::fastFree(pxlWRCount);

    return adjustRangeMapHeight( mRngMap, modSz);
}   // end adjustRangeMap



// private
cv::Mat_<float> RangeMapAdjuster::adjustRangeMapHeight( const cv::Mat_<float> &rngMap, const cv::Size2f &modSz) const
{
    const int rows = rngMap.rows;
    const int halfRows = rows/2;
    const int cols = rngMap.cols;
    const int halfCols = cols/2;

    cv::Mat_<float> hRngMap = cv::Mat_<float>( rngMap.size());

    float* curHRng = (float*)cv::fastMalloc(cols * sizeof(float));
    int* pxlHCount = (int*)cv::fastMalloc(cols * sizeof(int));
    for ( int i = 0; i < cols; ++i)
    {
        curHRng[i] = depthMax_;
        pxlHCount[i] = 0;
    }   // end for

    for ( int i = 0; i < rows; ++i)
    {
        const float *rngRow = rngMap.ptr<float>(i);
        float *rngRowOut = hRngMap.ptr<float>(i);

        for ( int j = 0; j < cols; ++j)
        {
            float depth = std::min( depthMax_, rngRow[j]);
            if ( depth == 0)
                depth = depthMax_;

            if ( depth < curHRng[j])
            {
                curHRng[j] = depth;
                pxlHCount[j] = (int)(modSz.height/depth * halfRows + 0.5);
            }   // end if

            pxlHCount[j]--;
            if (pxlHCount[j] <= 0)
                curHRng[j] = depth; // Reset

            rngRowOut[j] = curHRng[j];
        }   // end for - cols
    }   // end for - rows

    cv::fastFree(curHRng);
    cv::fastFree(pxlHCount);

    return hRngMap;
}   // end adjustRangeMapHeight


