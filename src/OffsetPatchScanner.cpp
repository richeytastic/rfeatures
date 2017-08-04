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

#include "OffsetPatchScanner.h"
using RFeatures::OffsetPatchScanner;
using RFeatures::Patch;
using RFeatures::OffsetPatch;
#include <cassert>


OffsetPatchScanner::OffsetPatchScanner( const cv::Mat_<float>& rm, const std::vector<Patch>& patches, int stepSz)
    : _rngMap(rm), _containingRect(0,0,rm.cols,rm.rows), _patches(patches), _stepSz(stepSz)
{
    _offsetPatches.resize(_patches.size());
}   // end ctor


const std::list<OffsetPatch>& OffsetPatchScanner::getOffsetPatches( int i) const
{
    assert( i >= 0 && i < _offsetPatches.size());
    return _offsetPatches[i];
}   // end getOffsetPatches


void OffsetPatchScanner::scan( float minRng, float maxRng)
{
    int ncols = _rngMap.cols;
    int nrows = _rngMap.rows;
    const int hrows = nrows/2;
    const int hcols = ncols/2;
    int nc = ncols;
    if ( _rngMap.isContinuous())
    {
        nc *= nrows;
        nrows = 1;
    }   // end for

    const int npatches = (int)_patches.size();

    for ( int i = 0; i < nrows; ++i)
    {
        const float* rngRow = _rngMap.ptr<float>(i);
        for ( int j = 0; j < nc; ++j)
        {
            const float dpth = rngRow[j];
            if ( dpth <= 0 || dpth < minRng || dpth > maxRng)
                continue;

            // Calculate offsets for each patch
            for ( int p = 0; p < npatches; ++p)
            {
                const Patch& patch = _patches[p];

                OffsetPatch op;
                op.pxlRect.width = cvRound( hcols * patch.realDims.width/dpth);
                op.pxlRect.height = cvRound( hrows * patch.realDims.height/dpth);
                // If the pixel dimensions are too small, we reject it
                if ( op.pxlRect.width < patch.minPxlDims.width || op.pxlRect.height < patch.minPxlDims.height)
                    continue;

                // Set the pixel position of where the pixel dimensions were calculated from
                op.pxlPt.x = j % ncols;
                op.pxlPt.y = j / ncols + i;

                // Determine offset location of the patch
                op.pxlRect.x = op.pxlPt.x - cvRound( patch.propOffset.x * op.pxlRect.width);
                op.pxlRect.y = op.pxlPt.y - cvRound( patch.propOffset.y * op.pxlRect.height);

                // If this places the patch outside of bounds, we reject it
                if ( !RFeatures::isContained( _containingRect, op.pxlRect))
                    continue;

                _offsetPatches[p].push_back(op);    // Add to the list of offsets for this patch
            }   // end for

        }   // end for
    }   // end for
}   // end scan
