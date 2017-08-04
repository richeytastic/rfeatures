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

#include "IntensityIndexer.h"
using RFeatures::IntensityIndexer;
#include "ImageHistogram.h"
using RFeatures::ImageHistogram;
#include "FeatureUtils.h"
#include <cassert>
#include <cstdlib>


IntensityIndexer::IntensityIndexer( const cv::Mat& image, int numLevs) : _numLevs(numLevs)
{
    assert( image.depth() == CV_8U);
    cv::Mat_<byte> tmpImg = image;
    if ( image.channels() == 3)
    {
        const cv::Mat_<float> limg = RFeatures::getCIELabChannels( image)[0];
        limg.convertTo( tmpImg, CV_8U, 255./100, 0);
    }   // end else
    else if ( image.channels() != 1)
        assert(false);

    cv::equalizeHist( tmpImg, tmpImg);
    //RFeatures::showImage( tmpImg, "Intensity Indexer ctor image (reduced levels)", false);
    // tmpImg now ranges from 0 to 255, but it needs to be from 0 to numLevs-1
    _image = tmpImg * double(numLevs-1)/255;

    /*
    double mn, mx;
    cv::minMaxLoc( _image, &mn, &mx);
    assert( mn >= 0 && mx <= numLevs-1);
    */
}   // end ctor


//RFeatures::showImage( _image, "Rescaled - IntensityIndexer", false);

struct IntensityLevel
{
    float ncount;  // Normalised count (set after mapping)
    float count;   // Histogram count
    int val;    // Mapped value after sort
};  // end struct


int cmpIntensityLevels( const void* i1, const void* i2)
{
    const IntensityLevel* l1 = *(const IntensityLevel* const*)i1;
    const IntensityLevel* l2 = *(const IntensityLevel* const*)i2;
    if ( l1->count > l2->count) // Note reversal of sort order
        return -1;
    else if ( l1->count < l2->count)    // Note reversal of sort order
        return 1;
    return 0;
}   // end cmpIntensityLevels


#include <iostream>
cv::Mat_<byte> IntensityIndexer::calcMapping( const cv::Rect subRect, cv::Mat_<byte> mask) const
{
    cv::Mat subImg = _image;
    if ( subRect.area() > 0)
        subImg = _image(subRect);
    if ( mask.empty())
        mask = cv::Mat_<byte>::ones( subImg.size());
    assert( mask.rows == subImg.rows && mask.cols == subImg.cols);

    //using std::cerr;
    //using std::endl;

    const int numLevs = _numLevs;
    // Create the histogram of intensity values
    cv::SparseMat hist = ImageHistogram( _image, numLevs).calcSparseHistogram( subRect, mask, 0.0f, numLevs-1);
    //cv::Mat hist = ImageHistogram( _image, numLevs).calcHistogram( subRect, mask, 0, numLevs-1);

    // Create array of intensity frequencies..
    float maxCount = 0;
    IntensityLevel* levs = (IntensityLevel*)cv::fastMalloc(numLevs * sizeof(IntensityLevel));
    IntensityLevel** levPtrs = (IntensityLevel**)cv::fastMalloc(numLevs * sizeof(IntensityLevel*));

    //cerr << "Original histogram levels:" << endl;
    for ( int i = 0; i < numLevs; ++i)
    {
        levs[i].count = hist.ref<float>(i);
        //cerr << "|" << levs[i].count;
        if ( levs[i].count > maxCount)
            maxCount = levs[i].count;
        levPtrs[i] = &levs[i];
    }   // end for
    //cerr << "|" << endl;

    //cerr << "Sorted histogram levels (descending):" << endl;
    qsort( levPtrs, numLevs, sizeof(IntensityLevel*), cmpIntensityLevels); // .. and sort
    // levPtrs now in descending order of frequency - create the mapping to the new values
    for ( int i = 0; i < numLevs; ++i)
    {
        //cerr << "|" << levPtrs[i]->count;
        levPtrs[i]->val = i+1;  // New mapped value
        levPtrs[i]->ncount = levPtrs[i]->count/maxCount;
    }   // end for
    //cerr << "|" << endl;

    const float div = 255.0f/numLevs;
    const int rows = subImg.rows;
    const int cols = subImg.cols;
    cv::Mat_<byte> outMap( rows, cols);

    for ( int i = 0; i < rows; ++i)
    {
        const byte* valRow = subImg.ptr<byte>(i);
        const byte* maskRow = mask.ptr<byte>(i);
        byte* outRow = outMap.ptr<byte>(i);

        for ( int j = 0; j < cols; ++j)
        {
            const int v = valRow[j] - 1;
            if ( maskRow[j] == 0 || v < 0)
                outRow[j] = 0;
            else
                outRow[j] = byte(levs[v].val * div);
        }   // end for - cols
    }   // end for - rows

    cv::fastFree(levPtrs);
    cv::fastFree(levs);

    return outMap;
}   // end calcMapping



cv::Mat_<byte> IntensityIndexer::operator()( const cv::Rect subRect, cv::Mat_<byte> mask) const
{
    return calcMapping( subRect, mask);
}   // end operator()
