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

#include "AdaptiveDepthPatchScanner.h"
using RFeatures::AdaptiveDepthPatchScanner;
using RFeatures::PatchProcessor;
using RFeatures::PatchRanger;
#include <cassert>


AdaptiveDepthPatchScanner::AdaptiveDepthPatchScanner( const cv::Mat_<float> depthMap, const cv::Size2f& rps, PatchProcessor* px, int pxlStepSz)
    : _rngMap(depthMap), _patchRanger(depthMap), _rpatchSz(rps), _px(px), _pxlStepSz(pxlStepSz),
      _imgRct(0,0,depthMap.cols,depthMap.rows), _subRct( _imgRct)
{
}   // end ctor


void AdaptiveDepthPatchScanner::setPatchProcessor( PatchProcessor* px)
{
    _px = px;
}   // end setPatchProcessor



void AdaptiveDepthPatchScanner::setSubRegion( const cv::Rect rct)
{
    if ( rct.area() == 0) // Reset to whole image
        _subRct = _imgRct;
    else
    {
        assert( (rct & _imgRct) == rct);
        _subRct = rct;
    }   // end else
}   // end setSubRegion



PatchRanger::PatchRanger( const cv::Mat_<float> rmap) : _rngMap(rmap), _rct(0,0,rmap.cols, rmap.rows)
{}   // end ctor



float PatchRanger::minDepthAtPoint( const cv::Point& p) const
{
    return minDepthAtPoint( p.y, p.x);
}   // end minDepthAtPoint



float PatchRanger::minDepthAtPoint( int row, int col) const
{
    float minDepth = 0;
    if ( _rct.contains(cv::Point(row,col)))
    {
        minDepth = _rngMap.at<float>(row,col);
        if ( minDepth == 0) // If no depth at the point, don't bother looking at its neighbours
            return 0;

        if ( row - 2 >= 0)
        {
            const float* rowRng = _rngMap.ptr<float>(row-2);
            if ( col - 2 >= 0)
            {
                const float rng = rowRng[col-2];
                if ( rng > 0 && rng < minDepth)
                    minDepth = rng;
            }   // end if

            if ( col + 2 < _rct.width)
            {
                const float rng = rowRng[col+2];
                if ( rng > 0 && rng < minDepth)
                    minDepth = rng;
            }   // end if
        }   // end if

        if ( row - 1 >= 0)
        {
            const float* rowRng = _rngMap.ptr<float>(row-1);
            if ( col - 1 >= 0)
            {
                const float rng = rowRng[col-1];
                if ( rng > 0 && rng < minDepth)
                    minDepth = rng;
            }   // end if

            if ( col + 1 < _rct.width)
            {
                const float rng = rowRng[col+1];
                if ( rng > 0 && rng < minDepth)
                    minDepth = rng;
            }   // end if
        }   // end if

        if ( row + 1 >= 0)
        {
            const float* rowRng = _rngMap.ptr<float>(row+1);
            if ( col - 1 >= 0)
            {
                const float rng = rowRng[col-1];
                if ( rng > 0 && rng < minDepth)
                    minDepth = rng;
            }   // end if

            if ( col + 1 < _rct.width)
            {
                const float rng = rowRng[col+1];
                if ( rng > 0 && rng < minDepth)
                    minDepth = rng;
            }   // end if
        }   // end if

        if ( row + 2 >= 0)
        {
            const float* rowRng = _rngMap.ptr<float>(row+2);
            if ( col - 2 >= 0)
            {
                const float rng = rowRng[col-2];
                if ( rng > 0 && rng < minDepth)
                    minDepth = rng;
            }   // end if

            if ( col + 2 < _rct.width)
            {
                const float rng = rowRng[col+2];
                if ( rng > 0 && rng < minDepth)
                    minDepth = rng;
            }   // end if
        }   // end if

        if ( minDepth == FLT_MAX)
            minDepth = 0;
    }   // end if

    return minDepth;
}   // end minDepthAtPoint



float PatchRanger::calcPatchRect( const cv::Point& p, const cv::Size2f& realSz, cv::Rect& patchRect) const
{
    return calcPatchRect( p.y, p.x, realSz, patchRect);
}   // end calcPatchRect



float PatchRanger::calcPatchRect( int py, int px, const cv::Size2f& realSz, cv::Rect& patchRect) const
{
    if ( !_rct.contains(cv::Point(px, py)))
        return 0;

    patchRect.x = px;
    patchRect.y = py;
    patchRect.width = 0;
    patchRect.height = 0;

    const float depth = _rngMap.at<float>(py, px);
    const float hcols = float(_rngMap.cols)/2;
    const float hrows = float(_rngMap.rows)/2;
    if ( depth > 0)
    {
        patchRect.width = cvRound( hcols * realSz.width/depth);
        patchRect.height = cvRound( hrows * realSz.height/depth);
        patchRect.x = px - patchRect.width/2;
        patchRect.y = py - patchRect.height/2;
    }   // end if

    return depth;
}   // end calcPatchRect



float PatchRanger::calcQuadrilateral( const cv::Point& p, const cv::Size2f& realSz, vector<cv::Point>& quad) const
{
    float d0 = 0;   // Depth at p
    float d1 = 0;   // Depth one pixel below p
    if (_rct.contains(p) && p.y < _rct.height-1)
    {
        d0 = _rngMap.at<float>(p);
        d1 = _rngMap.at<float>(cv::Point(p.x,p.y+1));
    }   // end if

    if ( d0 <= 0 || d1 <= 0)
        return 0;
  
    const float F = float(_rct.height)/2;    // Assumes 90 degree FOV
    const float p0 = fabs(p.y - F);
    const float p1 = fabs(p.y+1 - F);

    const float h0 = d0*p0/F;
    const float h1 = d1*p1/F;

    const float h = (d1*p1 - d0*p0)/F;
    const float w = d0 - d1;
    const float gam = sqrt(w*w + h*h);

    const float expFact = realSz.height/2*gam;

    const float nd0 = d0 + expFact*w;
    const float nd1 = d1 - expFact*w;
    const float nh1 = h1 + expFact*h;
    const float nh0 = h0 - expFact*h;

    const float rowDiff = fabs(F*nh1/nd1 - F*nh0/nd0);

    const cv::Point pTop( p.x, int(float(p.y) - rowDiff/2));
    const cv::Point pBot( p.x, int(float(p.y) + rowDiff/2));

    if ( !_rct.contains( pTop) || !_rct.contains( pBot))
        return 0;

    cv::Rect topRct, botRct;
    calcPatchRect( pTop, realSz, topRct);
    calcPatchRect( pBot, realSz, botRct);

    const cv::Point topLeft( pTop.x - topRct.width/2, pTop.y);
    const cv::Point topRight( pTop.x + topRct.width/2, pTop.y);
    const cv::Point botLeft( pBot.x - botRct.width/2, pBot.y);
    const cv::Point botRight( pBot.x + botRct.width/2, pBot.y);
    // Clockwise addition from top left
    quad.push_back(topLeft);
    quad.push_back(topRight);
    quad.push_back(botRight);
    quad.push_back(botLeft);

    return d0;
}   // end calcQuadrilateral



float PatchRanger::pointDepth( const cv::Point& p) const
{
    if ( _rct.contains(p))
        return _rngMap.at<float>(p);
    return 0;
}   // end pointDepth



cv::Size2f PatchRanger::operator()( const cv::Rect& pxlRect, bool fromMidBase) const
{
    cv::Point cpt = RFeatures::calcPixelCentre(pxlRect);
    if ( fromMidBase)
        cpt.y = pxlRect.y + pxlRect.height;
    const float fdepth = pointDepth( cpt);
    const float hfoclen = float(_rngMap.rows)/2;
    const float wfoclen = float(_rngMap.cols)/2;
    return cv::Size2f( (fdepth * pxlRect.width) / wfoclen, (fdepth * pxlRect.height) / hfoclen);
}   // end operator()


// private
void AdaptiveDepthPatchScanner::scan( const cv::Mat_<byte> mask, float minRng, float maxRng) const
{
    const cv::Rect subRect = _subRct;
    assert( mask.cols == subRect.width && mask.rows == subRect.height);
    const int rows = subRect.height;
    const int cols = subRect.width;

    const int rowStep = _pxlStepSz;
    const int colStep = _pxlStepSz;
    //const int nrows = rows / _pxlStepSz;
    const int rowrem = rows % _pxlStepSz;
    //const int ncols = cols / _pxlStepSz;
    const int colrem = cols % _pxlStepSz;
    const int rowst = rowrem/2;
    const int colst = colrem/2;

    PatchProcessor* px = _px;
    assert( px != NULL);
    cv::Point p;
    cv::Rect patchRect;
    for ( int i = rowst; i < rows; i += rowStep)
    {
        const byte* maskRow = NULL;
        if ( !mask.empty())
            maskRow = mask.ptr<byte>(i);

        for ( int j = colst; j < cols; j += colStep)
        {
            // Translate the relative point to absolute position in the image
            p.y = i + subRect.y;
            p.x = j + subRect.x;
            float depth = _patchRanger.pointDepth(p);

            if ( maskRow != NULL)
            {
                if (!maskRow[j])   // Skip zero mask values
                    continue;
            }   // end if
            else if ( depth < minRng || depth > maxRng)
                continue;

            _patchRanger.calcPatchRect( p, _rpatchSz, patchRect);
            // Only process the patch rectangle if it's wholly contained in the image
            if ( RFeatures::isWithin( _imgRct, patchRect))
                px->process( p, depth, patchRect);
        }   // end for
    }   // end for
}   // end scan


// public
void AdaptiveDepthPatchScanner::scan( const cv::Mat_<byte> mask) const
{
    scan( mask, 0, 0);
}   // end scan


// public
void AdaptiveDepthPatchScanner::scan( float minRng, float maxRng) const
{
    if ( minRng < 0)
        minRng = 0;

    assert( maxRng > minRng);
    scan( cv::Mat_<byte>(), minRng, maxRng);
}   // end scan



void AdaptiveDepthPatchScanner::at( const cv::Point p) const
{
    assert( _px != NULL);
    cv::Rect patchRect;
    const float depth = _patchRanger.calcPatchRect( p, _rpatchSz, patchRect);
    _px->process( p, depth, patchRect);
}   // end at
