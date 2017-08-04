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

#include "DepthSegmenter.h"
using RFeatures::DepthSegmenter;
#include <cstdlib>
#include <cmath>
#include <vector>


DepthSegmenter::DepthSegmenter( const cv::Mat_<float> rngMat, int depthLevels, float inlierFactor, float minRng, float maxRng)
    : _rngMat(rngMat), _depthLevels(depthLevels), _inlierFactor(inlierFactor)
{
    if ( _depthLevels < 1)
        _depthLevels = 1;
    if ( _inlierFactor < 1)
        _inlierFactor = 1;

    if ( minRng < 0)
        minRng = 0;
    if ( maxRng < 0)
        maxRng = 0;

    if ( maxRng > minRng)
    {
        _minRng = minRng;
        _maxRng = maxRng;
    }   // end if
    else
    {
        double minr, maxr;
        cv::minMaxLoc( rngMat, &minr, &maxr);
        _minRng = float(minr);
        _maxRng = float(maxr);
    }   // end else
}   // end ctor


//#include <iostream>
//using std::cerr;
//using std::endl;
void DepthSegmenter::setMinMaxFromSubRect( const cv::Rect& subRect)
{
    const cv::Mat_<float> subMat = _rngMat(subRect);
    double minr, maxr;
    cv::minMaxLoc( subMat, &minr, &maxr);
    _minRng = float(minr);
    _maxRng = float(maxr);
}   // end setMinMaxFromSubRect



cv::Mat_<byte> DepthSegmenter::operator()( const cv::Rect subRect, cv::Mat_<float> mask) const
{
    return calcMostCommonDepthMask( subRect, mask);
}  // end operator()



cv::Mat_<byte> DepthSegmenter::calcMostCommonDepthMask( const cv::Rect subRect, cv::Mat_<float> mask) const
{
    cv::Mat_<float> subMat = _rngMat;
    if ( subRect.area() > 0)
        subMat = _rngMat(subRect);
    if ( mask.empty())
        mask = cv::Mat_<float>::ones(subMat.size());
    assert( mask.rows == subMat.rows && mask.cols == subMat.cols);

    // Find the most common depth 
    int *bins = (int*)calloc( _depthLevels, sizeof(int));   // Zero'd
    float *means = (float*)calloc( _depthLevels, sizeof(float));    // Zero'd within bin depth means
    std::vector< std::vector<float> > dvals( _depthLevels);

    const double rngDelta = _maxRng - _minRng;

    int topIdx = 0; // Remember top index (most hits)
    int maxCnt = 0;
    const int rows = subMat.rows;
    const int cols = subMat.cols;
    for ( int i = 0; i < rows; ++i)
    {
        const float *pxRow = subMat.ptr<float>(i);
        const float *maskRow = mask.ptr<float>(i);

        for ( int j = 0; j < cols; ++j)
        {
            if ( maskRow[j])    // Ignore zero values
            {
                const float depth = pxRow[j];
                if ( depth <= 0)
                    continue;

                const int b = binVal( bins, means, depth, rngDelta);
                if ( b < 0) // Out of range so continue
                    continue;

                dvals[b].push_back(depth);  // Store the depth value itself for later std-dev calc
                if ( bins[b] > maxCnt)
                {
                    topIdx = b;
                    maxCnt = bins[b];
                }   // end if
            }   // end if
        }   // end for - cols
    }   // end for - rows

    /*
    for ( int i = 0; i < _depthLevels; ++i)
        cerr << "Bin " << i << ": " << bins[i] << endl;
    cerr << "Top bin = " << topIdx << endl;
    */
    free(bins);

    const float meanDepth = means[topIdx];    // Mean of most common depth values
    //free(means);

    // Calculate the std-dev for the most common depth interval
    const std::vector<float>& vds = dvals[topIdx];
    const int sz = (int)vds.size();
    double sumSqDiffs = 0;
    for ( int i = 0; i < sz; ++i)
        sumSqDiffs += pow(vds[i]-meanDepth,2);
    const double stddev = sqrt(sumSqDiffs/sz);
    _lastStdDev = float(stddev);

    // Identify the pixels closest to meanDepth within c*stddev
    const float withinRng = _inlierFactor * _lastStdDev;

    float minDepth = FLT_MAX;
    float maxDepth = 0;
    float totDepth = 0;
    int pxCount = 0;

    cv::Mat_<byte> outMat( rows, cols);
    for ( int i = 0; i < rows; ++i)
    {
        byte* outRow = outMat.ptr<byte>(i);
        const float* inRow = subMat.ptr<float>(i);
        for ( int j = 0; j < cols; ++j)
        {
            const float depth = inRow[j];
            if ( depth > 0 && (fabs(depth - meanDepth) < withinRng))
            {
                outRow[j] = 0xff;
                totDepth += depth;
                pxCount++;
                if ( depth < minDepth)
                    minDepth = depth;
                if ( depth > maxDepth)
                    maxDepth = depth;
            }   // end if
            else
                outRow[j] = 0;
        }   // end for
    }   // end for

    _lastMinDepth = minDepth;
    _lastMaxDepth = maxDepth;
    _lastAvgDepth = totDepth/pxCount;

    //cv::fastFree(dvals);
    return outMat;
}   // end calcMostCommonDepthMask



int DepthSegmenter::binVal( int* bins, float* means, double v, double vRng) const
{
    v -= _minRng;
    const int b = int(v/vRng * _depthLevels);
    if ( b >= _depthLevels || b < 0) // Ignore value if out of range
        return -1;
    bins[b]++;
    means[b] = float(((bins[b]-1) * (double)means[b] + v + _minRng) / bins[b]);
    return b;
}   // end binVal



void DepthSegmenter::getLastStats( float* lmin, float* lmax, float* lavg, float* lsdv) const
{
    if ( lmin != NULL) *lmin = _lastMinDepth;
    if ( lmax != NULL) *lmax = _lastMaxDepth;
    if ( lavg != NULL) *lavg = _lastAvgDepth;
    if ( lsdv != NULL) *lsdv = _lastStdDev;
}   // end getLastStats
