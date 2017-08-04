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

#include "AdaptiveDepthSegmenter.h"
using RFeatures::AdaptiveDepthSegmenter;
#include "FeatureUtils.h"
#include <cmath>
#include <vector>
using std::vector;
#include <algorithm>


AdaptiveDepthSegmenter::AdaptiveDepthSegmenter( const cv::Mat_<float> dmap, const cv::Size2f& rps, float threshVal)
    : _rngImg(dmap), _rpatchSz(rps), _threshVal(threshVal)
{
}   // end ctor



cv::Mat_<byte> AdaptiveDepthSegmenter::filter( float minRng, float maxRng)
{
    _outImg = cv::Mat_<byte>::zeros( _rngImg.size());
    AdaptiveDepthPatchScanner adps( _rngImg, _rpatchSz, this);
    adps.scan( 1, maxRng);
    // Now dilate white parts on grey sections of the image
    return _outImg;
}   // end filter


/*
// Calculates a measure of homogenity of variance.
void AdaptiveDepthSegmenter::process( const cv::Point& p, float pdepth, const cv::Rect& patchRct)
{
    const cv::Mat_<float> rngMap = _rngImg;

    const int y0 = std::max<int>( patchRct.y, 0);
    const int x0 = std::max<int>( patchRct.x, 0);
    const int y1 = std::min<int>( y0 + patchRct.height - 1, _outImg.rows -1);
    const int x1 = std::min<int>( x0 + patchRct.width - 1, _outImg.cols -1);

    float d[9];
    d[0] = rngMap.at<float>(y0, x0);
    d[1] = rngMap.at<float>(y0, p.x);
    d[2] = rngMap.at<float>(y0, x1);

    d[3] = rngMap.at<float>(p.y, x0);
    d[4] = pdepth;
    d[5] = rngMap.at<float>(p.y, x1);

    d[6] = rngMap.at<float>(y1, x0);
    d[7] = rngMap.at<float>(y1, p.x);
    d[8] = rngMap.at<float>(y1, x1);

    float m = 0;    // Calculate the mean
    for ( int i = 0; i < 9; ++i)
        m += d[i];
    m /= 9;

    // Group the points into two clusters either side of the mean
    vector<float> c0, c2;


    // Get the variance of each point from the cluster mean
    const float v0 = pow(r0c0 - m,2);
    const float v1 = pow(r0c1 - m,2);
    const float v2 = pow(r0c2 - m,2);
    const float v3 = pow(r1c0 - m,2);
    const float v4 = pow(r1c1 - m,2);
    const float v5 = pow(r1c2 - m,2);
    const float v6 = pow(r2c0 - m,2);
    const float v7 = pow(r2c1 - m,2);
    const float v8 = pow(r2c2 - m,2);

    const float vm = (v0 + v1 + v2
                    + v3 + v4 + v5
                    + v6 + v7 + v8) / 9;
}   // end process
*/




void AdaptiveDepthSegmenter::process( const cv::Point& p, float pdepth, const cv::Rect& patchRct)
{
    const cv::Mat_<float> rngMap = _rngImg;

    const int y0 = std::max<int>( patchRct.y, 0);
    const int x0 = std::max<int>( patchRct.x, 0);
    const int y1 = std::min<int>( y0 + patchRct.height - 1, _outImg.rows -1);
    const int x1 = std::min<int>( x0 + patchRct.width - 1, _outImg.cols -1);

    // Get depth values from nine points around the rectangle
    cv::Point pts[9];
    pts[0] = cv::Point( x0, y0);
    pts[1] = cv::Point( p.x, y0);
    pts[2] = cv::Point( x1, y0);

    pts[3] = cv::Point( x0, p.y);
    pts[4] = p;
    pts[5] = cv::Point( x1, p.y);

    pts[6] = cv::Point( x0, y1);
    pts[7] = cv::Point( p.x, y1);
    pts[8] = cv::Point( x1, y1);

    float d[9];
    for ( int i = 0; i < 9; ++i)
        d[i] = rngMap.at<float>( pts[i]);

    // Calculate the mean depth and find the point having nearest depth (but not zero)
    int npi = 4;    // Nearest point index
    float m = 0;
    for ( int i = 0; i < 9; ++i)
    {
        m += d[i];
        if (( d[i] > 0) && (d[i] < d[npi]))
            npi = i;
    }   // end for
    m /= 9;

    float v[9]; // Calculate the individual depth variances
    float vm = 0;
    for ( int i = 0; i < 9; ++i)
    {
        v[i] = pow(d[i] - m,2);
        vm += v[i];
    }   // end for
    vm /= 9;    // Mean of variance of depth

    // Calculate a measure of the non-homogenity of variance (how disimilar the variances are)
    float vs[9];
    float vsm = 0;
    for ( int i = 0; i < 9; ++i)
    {
        vs[i] = sqrt(pow(v[i] - vm,2));
        vsm += vs[i];
    }   // end for
    vsm /= 9;
    const float t = vsm;

    // Smaller values of t denote edges

    if ( t > _threshVal)
    {
        _outImg.at<byte>(p) = 0xff;
        _outImg.at<byte>(pts[npi]) = 0xff;
    }   // end if
    else
    {
        _outImg.at<byte>(p) = 120;
        _outImg.at<byte>(pts[npi]) = 120;
    }   // end else
}   // end process



/*
void AdaptiveDepthSegmenter::process( const cv::Point& p, float pdepth, const cv::Rect& patchRct)
{
    const cv::Mat_<float> rngMap = _rngImg;
    const int rowMax = patchRct.y + patchRct.height;
    const int colMax = patchRct.x + patchRct.width;

    int pxlCount = 0;
    float minVal = pdepth;
    float maxVal = pdepth;
    cv::Point minPt = p;

    // Collect the > 0 depth values from each pixel in the patch rectangle
    for ( int i = patchRct.y; i < rowMax; ++i)
    {
        if ( i < 0) // Ignore rows above the first image row
           continue;

        else if ( i >= rngMap.rows) // Ignore rows >= the last image row
            break;

        const float* rngRow = rngMap.ptr<float>(i);

        for ( int j = patchRct.x; j < colMax; ++j)
        {
            if ( j < 0) // Ignore columns to the left of the first
                continue;

            else if ( j >= rngMap.cols) // Ignore columns to the right of the last
                break;

            const float dv = rngRow[j];

            if ( dv > 0)
                pxlCount++;

            if ( dv < minVal)
            {
                minVal = dv;
                if ( dv > 0)
                {
                    minPt.y = i;
                    minPt.x = j;
                }   // end if
            }   // end if

            if ( dv > maxVal)
                maxVal = dv;
        }   // end for
    }   // end for

    if ( pxlCount == 0)
        return;

    //const float tval = pow(pdepth - minVal,2) + pow(pdepth - maxVal,2) + pow(maxVal - minVal,2);
    //const float tval = fabsf(pow( pow(pdepth - minVal,2) - pow(pdepth - maxVal,2), 2) - pow(maxVal - minVal,2));
    //const float tval = pow( pow(pdepth - minVal,2) - pow(pdepth - maxVal,2), 2);
    const float tval = pow(minVal - 2*pdepth - maxVal,2);
    if ( tval > _threshVal)
        _outImg.at<byte>(minPt) = 0xff;
}   // end process
*/




