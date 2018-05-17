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

#ifndef RFEATURES_RANGE_PATCH_SCANNER_H
#define RFEATURES_RANGE_PATCH_SCANNER_H

#include "AdaptiveDepthPatchScanner.h"
#include <boost/shared_ptr.hpp>


namespace RFeatures {

class rFeatures_EXPORT RangePatchScanner : public PatchProcessor
{
public:
    typedef boost::shared_ptr<RangePatchScanner> Ptr;
    static Ptr create( const cv::Mat_<float>& rngMap, float minRng=0, float maxRng=FLT_MAX);

    RangePatchScanner( const cv::Mat_<float>& rngMap, float minRng=0, float maxRng=FLT_MAX);
    virtual ~RangePatchScanner();

    const cv::Mat_<byte>& getMask() const { return _rngMask;}
    const cv::Mat_<int>& getMaskIntegralImage() const { return _rngMaskIntImg;}

    double getVerticalGradientSum( const cv::Rect& rct) const;
    double getVerticalGradientVarianceSum( const cv::Rect& rct) const;
    double getHorizontalGradientSum( const cv::Rect& rct) const;
    double getHorizontalGradientVarianceSum( const cv::Rect& rct) const;

    void produceRangeMaps( const cv::Size2f& realPatchSize);    // Function to produce the following four maps
    const cv::Mat_<float>& getVerticalGradientMap() const { return _vRngChng;}
    const cv::Mat_<float>& getHorizontalGradientMap() const { return _hRngChng;}
    const cv::Mat_<float>& getVerticalGradientVarianceMap() const { return _vVarRngChng;}
    const cv::Mat_<float>& getHorizontalGradientVarianceMap() const { return _hVarRngChng;}

protected:
    virtual void process( const cv::Point& pt, float depthVal, const cv::Rect& patchRect);

private:
    const AdaptiveDepthPatchScanner* _adps;
    const cv::Mat_<float> _rngMap;

    cv::Mat_<byte> _rngMask;
    cv::Mat_<int> _rngMaskIntImg;

    cv::Mat_<float> _vChngIntImg;
    cv::Mat_<double> _vChngSqIntImg;
    cv::Mat_<float> _hChngIntImg;
    cv::Mat_<double> _hChngSqIntImg;

    cv::Mat_<float> _vRngChng;
    cv::Mat_<float> _hRngChng;
    cv::Mat_<float> _vVarRngChng;
    cv::Mat_<float> _hVarRngChng;
};  // end class

}   // end namespace
#endif
