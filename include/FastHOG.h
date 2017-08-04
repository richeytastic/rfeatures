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

#pragma once
#ifndef RFEATURES_FAST_HOG_H
#define RFEATURES_FAST_HOG_H

/**
 * FAST HOG (Histograms of Oriented Gradients)!
 * (Probably no faster than regular HOG).
 *
 * Richard Palmer
 * November 2013
 **/

#include "FeatureOperator.h"
#include "FeatureUtils.h"
#include <boost/shared_ptr.hpp>

namespace RFeatures
{

class rFeatures_EXPORT FastHOG : public RFeatures::FeatureOperator
{
public:
    typedef boost::shared_ptr<FastHOG> Ptr;

    // img: Source image
    // nbins: Length of histogram to bin into
    // pxlWin: Local pixel window dimensions to group contrast differences over
    // fvDims: Feature scaling size (do not provide if not scaling features to fixed size)
    FastHOG( const cv::Mat img, int nbins, const cv::Size pxlWin, const cv::Size fvDims=cv::Size(0,0));
    FastHOG( const cv::Mat_<float>& I_x, const cv::Mat_<float>& I_y, int nbins, const cv::Size pxlWin,
                                                                                const cv::Size fvDims=cv::Size(0,0));

    // Values in all returned matrices in range [0,1]
    cv::Mat getHOGs() const { return _hogs;}    // CV_32FC(_nbins)
    cv::Mat_<float> getMaxAngles() const { return _maxAngles;}
    cv::Mat_<float> getMaxMags() const { return _maxMags;}

    // Returns a HOG image with nbins channels of single precision floats in range [0,1].
    // Can be converted to cv::Mat_<cv::Vec_<float, nbins> >.
    static cv::Mat makeHOGs( const cv::Mat img, int nbins, const cv::Size pxlWin);
    static cv::Mat makeHOGs( const cv::Mat_<float>& I_x, const cv::Mat_<float>& I_y, int nbins, const cv::Size pxlWin);

protected:
    virtual void getSampleChannels( const cv::Rect&, vector<cv::Mat>&) const;

private:
    const int _nbins;
    const cv::Size _pxlWin;

    cv::Mat _hogs;
    cv::Mat_<float> _maxAngles;
    cv::Mat_<float> _maxMags;

    cv::Mat makeHOGsFromPxlBins( const cv::Mat pxlBins, const cv::Size pxlWin);
};  // end class

}   // end namespace

#endif
