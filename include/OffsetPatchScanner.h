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
#ifndef RFEATURES_OFFSET_PATCH_SCANNER_H
#define RFEATURES_OFFSET_PATCH_SCANNER_H

#include "FeatureUtils.h"
#include <list>


namespace RFeatures
{

struct Patch
{
    cv::Size minPxlDims;    // Minimum allowed size of a patch in pixels - patches smaller than this are ignored.
    cv::Size2f realDims;    // Real dimensions of patch (in same metrics as the given range map)
    cv::Point2f propOffset; // Offset of patch to reference point (size calculation point) (set as 0.5,0.5 for patch centre)
                            // Offset is always given as proportion of the patch's real dimensions.
};  // end struct


// OffsetPatch specifies the pixel size and location of a patch (pxlRect)
// and the point in the image from which its position was calculated (pxlPt).
struct OffsetPatch
{
    cv::Rect pxlRect;
    cv::Point pxlPt;
};  // end struct


// Obtain patches of varying pixel dimensions at positions over the given range image for specified patches.
// Default step is every pixel (stepSz).
class rFeatures_EXPORT OffsetPatchScanner
{
public:
    OffsetPatchScanner( const cv::Mat_<float>& rngMap, const std::vector<Patch>& patches, int stepSz=1);

    void scan( float minRng=0, float maxRng=FLT_MAX);   // Scans whole range map if no parameters set

    const std::list<OffsetPatch>& getOffsetPatches( int patchId) const;

private:
    const cv::Mat_<float> _rngMap;
    const cv::Rect _containingRect;
    const std::vector<Patch>& _patches;
    const int _stepSz;

    std::vector<std::list<OffsetPatch> > _offsetPatches;
};  // end class

}   // end namespace
#endif
