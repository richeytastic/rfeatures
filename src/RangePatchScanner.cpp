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

#include <RangePatchScanner.h>
#include <cassert>
#include <cmath>
using RFeatures::RangePatchScanner;

#ifdef WIN32
    #define ISNAN _isnan
#else
    #define ISNAN std::isnan
#endif


RangePatchScanner::Ptr RangePatchScanner::create( const cv::Mat_<float>& rm, float minRng, float maxRng)
{
    return Ptr( new RangePatchScanner( rm, minRng, maxRng));
}   // end create


RangePatchScanner::RangePatchScanner( const cv::Mat_<float>& rm, float minRng, float maxRng)
    : _adps(NULL), _rngMap(rm)
{
    _rngMaskIntImg = RFeatures::createMaskIntegralImage( _rngMap, minRng, maxRng, _rngMask);
    // Create integral images of the vertical and horizontal change and squares of these
    cv::Mat_<float> hchng, vchng;
    RFeatures::createChangeMaps( _rngMap, hchng, vchng, false/*don't use absolute value*/, _rngMask);
    cv::integral( vchng, _vChngIntImg, _vChngSqIntImg, CV_32F);
    cv::integral( hchng, _hChngIntImg, _hChngSqIntImg, CV_32F);
}   // end ctor


RangePatchScanner::~RangePatchScanner()
{
    if ( _adps != NULL)
        delete _adps;
}   // end dtor


// public
void RangePatchScanner::produceRangeMaps( const cv::Size2f& realSz)
{
    if ( _adps != NULL)
        delete _adps;
    _adps = new RFeatures::AdaptiveDepthPatchScanner( _rngMap, realSz, this, 1);
    const cv::Size& sz = _rngMask.size();
    _hRngChng = cv::Mat_<float>::zeros( sz);
    _vRngChng = cv::Mat_<float>::zeros( sz);
    _hVarRngChng = cv::Mat_<float>::zeros( sz);
    _vVarRngChng = cv::Mat_<float>::zeros( sz);
    _adps->scan(_rngMask);
}   // end doscan



// protected - virtual
void RangePatchScanner::process( const cv::Point& pt, float dval, const cv::Rect& prct)
{
    assert( prct.x >= 0);
    assert( prct.y >= 0);
    assert( prct.width <= _rngMask.cols);
    assert( prct.height <= _rngMask.rows);

    const int vrngCnt = RFeatures::getIntegralImageSum<int>( _rngMaskIntImg, prct); // Number of valid points within range over this patch
    assert( vrngCnt >= 0);
    if ( vrngCnt == 0)
    {
        _hRngChng.at<float>(pt) = 0;
        _vRngChng.at<float>(pt) = 0;
        _hVarRngChng.at<float>(pt) = 0;
        _vVarRngChng.at<float>(pt) = 0;
        return;
    }   // end if
    
    // Change in range. Positive means range values increasing (moving further away) from top to bottom, negative values
    // means range values decreasing (coming closer) from top to bottom (like a road surface for example).
    const float vc = RFeatures::getIntegralImageSum<float>( _vChngIntImg, prct) / vrngCnt;
    const float hc = RFeatures::getIntegralImageSum<float>( _hChngIntImg, prct) / vrngCnt;
    _hRngChng.at<float>(pt) = hc;
    _vRngChng.at<float>(pt) = vc;

    // Get the variance in the rate of change of range in the vertical and horizontal directions
    const double vvar = RFeatures::calcVariance<float>( _vChngIntImg, _vChngSqIntImg, prct, _rngMaskIntImg);
    const double hvar = RFeatures::calcVariance<float>( _hChngIntImg, _hChngSqIntImg, prct, _rngMaskIntImg);
    _hVarRngChng.at<float>(pt) = (float)hvar;
    _vVarRngChng.at<float>(pt) = (float)vvar;
}   // end process


// public
double RangePatchScanner::getHorizontalGradientSum( const cv::Rect& rct) const
{
    const int vrngCnt = RFeatures::getIntegralImageSum<int>( _rngMaskIntImg, rct);
    assert( vrngCnt >= 0);
    if ( vrngCnt == 0)
        return 0;

    const double v = RFeatures::getIntegralImageSum<float>( _hChngIntImg, rct) / vrngCnt;
    assert(!ISNAN(v));
    return v;
}   // end getHorizontalGradientSum



// public
double RangePatchScanner::getVerticalGradientSum( const cv::Rect& rct) const
{
    const int vrngCnt = RFeatures::getIntegralImageSum<int>( _rngMaskIntImg, rct);
    assert( vrngCnt >= 0);
    if ( vrngCnt == 0)
        return 0;

    const double v = RFeatures::getIntegralImageSum<float>( _vChngIntImg, rct) / vrngCnt;
    assert(!ISNAN(v));
    return v;
}   // end getVerticalGradientSum



// public
double RangePatchScanner::getHorizontalGradientVarianceSum( const cv::Rect& rct) const
{
    const int vrngCnt = RFeatures::getIntegralImageSum<int>( _rngMaskIntImg, rct);
    assert( vrngCnt >= 0);
    if ( vrngCnt == 0)
        return 0;

    const double v = RFeatures::calcVariance<float>( _hChngIntImg, _hChngSqIntImg, rct, _rngMaskIntImg);
    assert(!ISNAN(v));
    return v;
}   // end getHorizontalGradientVarianceSum



// public
double RangePatchScanner::getVerticalGradientVarianceSum( const cv::Rect& rct) const
{
    const int vrngCnt = RFeatures::getIntegralImageSum<int>( _rngMaskIntImg, rct);
    assert( vrngCnt >= 0);
    if ( vrngCnt == 0)
        return 0;

    const double v = RFeatures::calcVariance<float>( _vChngIntImg, _vChngSqIntImg, rct, _rngMaskIntImg);
    assert(!ISNAN(v));
    return v;
}   // end getVerticalGradientVarianceSum
