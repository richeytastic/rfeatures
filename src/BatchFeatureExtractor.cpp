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

#include "BatchFeatureExtractor.h"
using RFeatures::BatchFeatureExtractor;
using RFeatures::FeatureExtractor;
#include <iostream>
#include <algorithm>
#include <cassert>

void checkExtractCtor( const FeatureExtractor::Ptr fx, const std::vector<cv::Mat>& extracts)
{
#ifndef NDEBUG
    std::cerr << " (WARNING) BatchFeatureExtractor is DEBUG --> using only a single thread!" << std::endl;
#endif
    assert( !extracts.empty());
    if ( !RFeatures::checkFXImageTypeMismatch( fx, extracts[0]))
        throw RFeatures::ImageTypeException( "[ERROR] BatchFeatureExtractor: image data type is not compatible with the provided feature extractor!");
}   // end checkExtractCtor


BatchFeatureExtractor::BatchFeatureExtractor( FeatureExtractor::Ptr fx, const std::vector<cv::Mat>& extracts, double scaleFactor)
    : _fx(fx), _scaleFactor(scaleFactor), _extracts(&extracts), _rextracts(NULL), _msecs(0)
{
    checkExtractCtor( fx, extracts);
}  // end ctor


void checkRectCtor( const FeatureExtractor::Ptr fx, const cv::Mat& img)
{
#ifndef NDEBUG
    std::cerr << " (WARNING) BatchFeatureExtractor is DEBUG --> using only a single thread!" << std::endl;
#endif
    assert( !img.empty());
    if ( !RFeatures::checkFXImageTypeMismatch( fx, img))
        throw RFeatures::ImageTypeException( "[ERROR] BatchFeatureExtractor: image data type is not compatible with the provided feature extractor!");
}   // end checkRectCtor


BatchFeatureExtractor::BatchFeatureExtractor( FeatureExtractor::Ptr fx, const cv::Mat& img, const vector<cv::Rect>& rextracts, double sf)
    : _fx(), _scaleFactor(sf), _extracts(NULL), _rextracts(&rextracts), _msecs(0)
{
    checkRectCtor( fx, img);
    const cv::Size newSz( int(img.cols * sf), int(img.rows * sf));  // Floors dims
    cv::Mat inm;
    cv::resize( img, inm, newSz);
    _fx = fx->preProcess(inm);
    assert( _fx != NULL);
}   // end ctor


void BatchFeatureExtractor::extractRectFeatures( int si, int count)  // Boost thread function
{
    const double sf = _scaleFactor;
    const FeatureExtractor::Ptr fx = _fx;

    const int fi = si + count;
    for ( int i = si; i < fi; ++i)
    {
        // Use the already pre-processed image which may have been resized prior to processing, so apply sf to rectangle too
        cv::Rect r = _rextracts->at(i);
        RFeatures::scale( r, sf);   // Scale r inplace
        _fvs[i] = fx->extract( r); // Will throw if r is too small for fx
    }   // end for
}   // end extractRectFeatures


void BatchFeatureExtractor::extractMatFeatures( int si, int count)  // Boost thread function
{
    const double sf = _scaleFactor;

    const FeatureExtractor::Ptr fx = _fx;
    const cv::Size minDims = fx->getMinSamplingDims();

    const int fi = si + count;
    for ( int i = si; i < fi; ++i)
    {
        cv::Mat xm = _extracts->at(i);
        // Resize xm by the scale factor (no change if sf == 1) and never
        // resize below minimum dimensions for the feature extractor.
        cv::Size rsz( std::max<int>( minDims.width, xm.cols * sf), std::max<int>( minDims.height, xm.rows  * sf));
        cv::Mat inm;
        cv::resize( xm, inm, rsz);
        const FeatureExtractor::Ptr fx0 = fx->preProcess(inm);
        _fvs[i] = fx0->extract();
    }   // end for
}   // end extractMatFeatures


const std::vector<cv::Mat_<float> >& BatchFeatureExtractor::extract()
{
    if ( _fvs.empty())
    {
        // Get the number of extracts to be processed
        int nextracts = 0;
        if ( _extracts)
            nextracts = (int)_extracts->size();
        if ( _rextracts)
            nextracts = (int)_rextracts->size();

        _fvs.resize( nextracts);

        boost::thread_group tgroup;
        int nthreads = 1;   // Only use a single thread if debugging
#ifdef NDEBUG
        nthreads = boost::thread::hardware_concurrency();
#endif
        const int segsz = nextracts / nthreads; // Base number of extracts each thread must process
        int segrm = nextracts % nthreads;   // Remainder

        _msecs = 0;
        rlib::CpuTimer timer( _msecs);

        int si = 0;
        for ( int i = 0; i < nthreads; ++i)
        {
            // Get the number of extracts to process on this thread
            int thisSeg = segsz;
            if ( segrm > 0)
            {
                thisSeg++;
                segrm--;
            }   // end if

            if ( _rextracts)
                tgroup.create_thread( boost::bind( &BatchFeatureExtractor::extractRectFeatures, this, si, thisSeg));
            else
                tgroup.create_thread( boost::bind( &BatchFeatureExtractor::extractMatFeatures, this, si, thisSeg));

            si += thisSeg;
        }   // end for

        tgroup.join_all();  // Threads also deleted
    }   // end if

    return _fvs;
}   // end extract


double BatchFeatureExtractor::msecs() const
{
    return _msecs;
}   // end msecs

