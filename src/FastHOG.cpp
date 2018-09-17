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

#include <FastHOG.h>
using RFeatures::FastHOG;
#include <cassert>
#include <cmath>
#include <algorithm>


void binValue( float y, float x, float* bins, int nbins)
{
    const double angle = (atan2(y, x) + CV_PI)/(2*CV_PI+1e-8); // in [0,1)
    // Get the bin in [0,nbins)
    const double rawBin = angle * nbins; // (rawBin in [0,nbins)
    int b1 = int(rawBin);

    // Find bin2 for bi-linear interpolation
    double delta = rawBin-b1-0.5; // Interpolate bin to left or right?
    int b2;
    if ( delta < 0)
    {
        b2 = b1 == 0 ? nbins-1 : b1-1;
        delta = -delta;
    }   // end if
    else
        b2 = b1 == nbins-1 ? 0 : b1+1;

    // + small error term to avoid div/0 errors later on
    const double mag = sqrt(y*y + x*x) + 1e-8;
    bins[b1] = float((1.-delta)*mag);  // Later weighted with 2D Gaussian of pxlWin size
    bins[b2] = float(delta*mag);   // Later weighted with 2D Gaussian of pxlWin size
}   // end binValue


cv::Mat makePixelBins( const cv::Mat_<float>& I_x, const cv::Mat_<float>& I_y, int nbins)
{
    const cv::Size imgSz = I_x.size();
    cv::Mat pxlBins = cv::Mat::zeros( imgSz, CV_32FC(nbins));
    for ( int i = 0; i < imgSz.height; ++i)
    {
        const float* yrow = I_y.ptr<float>(i);
        const float* xrow = I_x.ptr<float>(i);
        float* binsRow = pxlBins.ptr<float>(i);
        for ( int j = 0; j < imgSz.width; ++j)
            binValue( yrow[j], xrow[j], &binsRow[j*nbins], nbins);
    }   // end for - rows
    return pxlBins;
}   // end makePxlBins


cv::Mat makePixelBins( const cv::Mat& img, int nbins)
{
    const cv::Size imgSz = img.size();
    const int channels = img.channels();
    cv::Mat pxlBins = cv::Mat::zeros( imgSz, CV_32FC(nbins));
    for ( int i = 0; i < imgSz.height; ++i)
    {
        float* binsRow = pxlBins.ptr<float>(i);
        for ( int j = 0; j < imgSz.width; ++j)
        {
            // Find the largest magnitude change over all the channels
            double topMag = 0;
            double y = 0;
            double x = 0;
            for ( int k = 0; k < channels; ++k)
            {
                const double ygrad = RFeatures::calcVerticalGrad( img, i, j, k);
                const double xgrad = RFeatures::calcHorizontalGrad( img, i, j, k);
                const double mag = ygrad*ygrad + xgrad*xgrad;
                if ( mag >= topMag)
                {
                    topMag = mag;
                    y = ygrad;
                    x = xgrad;
                }   // end if
            }   // end for

            binValue( float(y), float(x), &binsRow[j*nbins], nbins);
        }   // end for
    }   // end for - rows
    return pxlBins;
}   // end makePixelBins


cv::Mat FastHOG::makeHOGsFromPxlBins( const cv::Mat pxlBins, const cv::Size pxlWin)
{
    const cv::Size imgSz = pxlBins.size();
    const int nbins = pxlBins.channels();
    // Consolidate orientation bins over pxlWin areas and weight with a 2D Gaussian
    const cv::Mat_<float> gfilter = RFeatures::make2DGaussian( pxlWin);

    cv::Mat hogs = cv::Mat::zeros( imgSz, CV_32FC(nbins));
    _maxAngles = cv::Mat_<float>::zeros( imgSz);
    _maxMags = cv::Mat_<float>::zeros( imgSz);

    const int halfWinHeight = pxlWin.height/2;
    const int halfWinWidth = pxlWin.width/2;

    for ( int i = 0; i < imgSz.height; ++i)
    {
        float* hogsRow = hogs.ptr<float>(i);
        float* anglesRow = _maxAngles.ptr<float>(i);
        float* maxHogRow = _maxMags.ptr<float>(i);

        for ( int j = 0; j < imgSz.width; ++j)
        {
            // Determine the pixel start (inclusive) and end (exclusive) indices for pxlWin centred at i,j
            int y = i - halfWinHeight;
            int x = j - halfWinWidth;
            int endy = y + pxlWin.height;
            if ( endy > imgSz.height)
                endy = imgSz.height;
            int endx = x + pxlWin.width;
            if ( endx > imgSz.width)
                endx = imgSz.width;

            // Set the offset indices into the Gaussian filter
            int gyoffset = 0;
            if ( y < 0)
                gyoffset = -y;

            int gxoffset = 0;
            if ( x < 0)
                gxoffset = -x;

            double sum = nbins*1e-8;
            float* hogVals = &hogsRow[j*nbins]; // nbins long array

            // Iterate over the intersecting area of pxlWin centred at i,j collecting Gaussian weighted bin values
            int gy = gyoffset;
            for ( y = i; y < endy; ++y, ++gy)
            {
                assert( gy >= 0 && gy <= pxlWin.height);

                const float* pxlBinsRow = pxlBins.ptr<float>(y);
                const float* g_row = gfilter.ptr<float>(gy);
                int gx = gxoffset;
                for ( x = j; x < endx; ++x, ++gx)
                {
                    assert( gx >= 0 && gx <= pxlWin.width);
                    // Get the Gaussian weight at this position corresponding to the 2D index into the sub window.
                    const double gweight = g_row[gx];
                    assert( gweight > 0 && gweight < 1);

                    const float* bins = &pxlBinsRow[x*nbins];
                    for ( int b = 0; b < nbins; ++b)
                    {
                        const double bval = gweight * bins[b];
                        assert( bval >= 0.0);
                        sum += float(bval);
                        hogVals[b] += float(bval);
                    }   // end for
                }   // end for - pxlWin columns within image
            }   // end for - pxlWin rows within image

            int maxBin = 0;
            float maxHOG = 0;
            for ( int b = 0; b < nbins; ++b)
            {
                hogVals[b] /= float(sum);
                if ( hogVals[b] > 1)
                    hogVals[b] = 1;
                assert( hogVals[b] >= 0 && hogVals[b] <= 1);
                if ( hogVals[b] > maxHOG)
                {
                    maxHOG = hogVals[b];
                    maxBin = b;
                }   // end if
            }   // end for 

            // Set the _maxAngles and _maxMags
            anglesRow[j] = float(maxBin+1)/nbins;
            maxHogRow[j] = maxHOG;
        }   // end for - cols
    }   // end for - rows

    return hogs;
}   // end makeHOGsFromPixelBins


// static
cv::Mat FastHOG::makeHOGs( const cv::Mat img, int nbins, const cv::Size pxlWin)
{
    FastHOG fhog( img, nbins, pxlWin);
    return fhog._hogs;
}   // end makeHOGs


// static
cv::Mat FastHOG::makeHOGs( const cv::Mat_<float>& I_x, const cv::Mat_<float>& I_y, int nbins, const cv::Size pxlWin)
{
    FastHOG fhog( I_x, I_y, nbins, pxlWin);
    return fhog._hogs;
}   // end makeHOGs


FastHOG::FastHOG( const cv::Mat img, int nbins, const cv::Size pxlWin, const cv::Size fvDims)
    : RFeatures::FeatureOperator( img.size(), fvDims), _nbins(nbins), _pxlWin(pxlWin)
{
    assert( img.size().width >= _pxlWin.width && img.size().height >= _pxlWin.height);
    const cv::Mat pxlBins = makePixelBins( img, _nbins);
    _hogs = makeHOGsFromPxlBins( pxlBins, _pxlWin);
}   // end ctor


FastHOG::FastHOG( const cv::Mat_<float>& I_x, const cv::Mat_<float>& I_y, int nbins, const cv::Size pxlWin, const cv::Size fvDims)
    : RFeatures::FeatureOperator( I_x.size(), fvDims), _nbins(nbins), _pxlWin(pxlWin)
{
#ifndef NDEBUG
    const cv::Size ixsz = I_x.size();
    assert( ixsz == I_y.size());
    assert( ixsz.width >= _pxlWin.width && ixsz.height >= _pxlWin.height);
#endif
    const cv::Mat pxlBins = makePixelBins( I_x, I_y, nbins);
    _hogs = makeHOGsFromPxlBins( pxlBins, pxlWin);
}   // end makeHOGs


void FastHOG::getSampleChannels( const cv::Rect& rct, vector<cv::Mat>& simgs) const
{
    vector<cv::Mat> hogChannels;
    cv::split( _hogs, hogChannels);
    std::for_each( std::begin( hogChannels), std::end( hogChannels), [&]( const cv::Mat& hogc){ simgs.push_back( hogc(rct));});
    simgs.push_back( _maxMags(rct));
    simgs.push_back( _maxAngles(rct));
}   // end getSampleChannels
