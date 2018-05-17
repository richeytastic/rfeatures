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

#ifndef RFEATURES_ADAPTIVE_DEPTH_PATCH_SCANNER_H
#define RFEATURES_ADAPTIVE_DEPTH_PATCH_SCANNER_H

/**
 * Scans a depth image in a continuous line given a real patch size, passing at each pixel
 * a rectangular patch calculated from this point according to the given dimensions.
 *
 * Even rows are scanned left to right, odd rows are scanned right to left.
 */

#include "FeatureUtils.h"

namespace RFeatures {

class AdaptiveDepthPatchScanner;


class rFeatures_EXPORT PatchProcessor
{
protected:
    // pt: the absolute x,y indices used to create patchRect.
    // patchRect: the rectangle dimensions calculated at pt.
    // NB patchRect may sit outside of the image dimensions!
    // The value of the depth at the point is also provided.
    virtual void process( const cv::Point& pt, float depthVal, const cv::Rect& patchRect) = 0;

    friend class AdaptiveDepthPatchScanner;
};  // end class


class rFeatures_EXPORT PointProcessor
{
protected:
    virtual void process( const cv::Point& pt, float depthVal) = 0;
};  // end class


class rFeatures_EXPORT PatchRanger
{
public:
    explicit PatchRanger( const cv::Mat_<float> depthMap);

    // Sets scaledRect to be the pixel region centred at point p in an image given
    // real patch dimensions (realSz) and the depth at p. Returns the depth.
    float calcPatchRect( const cv::Point& p, const cv::Size2f& realSz, cv::Rect& scaledRect) const;
    float calcPatchRect( int row, int col, const cv::Size2f& realSz, cv::Rect& scaledRect) const;

    float calcQuadrilateral( const cv::Point& p, const cv::Size2f& realSz, vector<cv::Point>& quad) const;

    // Return the minimum depth at point p over a diagonal cross 5 pixels high and 5 pixels wide centred at p.
    float minDepthAtPoint( const cv::Point& p) const;
    float minDepthAtPoint( int row, int col) const;

    float pointDepth( const cv::Point&) const;

    // Given a pixel rectangle into the image, returns the real world size.
    // Default measurement point is at the centre of the pxlRect, but a better
    // estimate may be obtained if measured at the base of the rectangle (if the object
    // is sitting on the ground) - in which case set fromMidBase to true.
    cv::Size2f operator()( const cv::Rect& pxlRect, bool fromMidBase=false) const;

private:
    const cv::Mat_<float> _rngMap;
    const cv::Rect _rct;
};  // end class



class rFeatures_EXPORT AdaptiveDepthPatchScanner
{
public:
    AdaptiveDepthPatchScanner( const cv::Mat_<float> depthMap, const cv::Size2f& realPatchSize, PatchProcessor* px=NULL, int pxlStepSz=1);

    void setPatchProcessor( PatchProcessor* px);

    // Do all subsequent scans over given sub rectangle of depthMap.
    // Call without parameter to reset to whole image.
    void setSubRegion( const cv::Rect rct=cv::Rect());

    // Scan only at pixels != 0
    void scan( const cv::Mat_<byte> mask) const;

    // Scan but ignore all values outside of [minRange, maxRange). maxRange must be > minRange.
    void scan( float minRange=0, float maxRange=FLT_MAX) const;

    // Don't scan, but cause PatchProcessor::process to be called using the
    // point p (ignores any set sub-region) as calculation reference for patchRect.
    void at( const cv::Point p) const;

private:
    const cv::Mat_<float> _rngMap;
    const PatchRanger _patchRanger;
    const cv::Size2f _rpatchSz;
    PatchProcessor* _px;
    const int _pxlStepSz;
    const cv::Rect _imgRct;
    cv::Rect _subRct;
    void scan( const cv::Mat_<byte>, float, float) const;
};  // end class

}   // end namespace

#endif
