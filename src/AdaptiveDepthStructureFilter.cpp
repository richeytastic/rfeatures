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

#include "AdaptiveDepthStructureFilter.h"
using RFeatures::AdaptiveDepthStructureFilter;
#include "FeatureUtils.h"
#include "View.h"
#include <cmath>
#include <list>
using std::list;
#include <algorithm>


AdaptiveDepthStructureFilter::AdaptiveDepthStructureFilter( const cv::Mat_<float> dmap, const cv::Size2f& rps)
    : _rngImg(dmap), _patchRanger(dmap), _rpatchSz(rps)
{
}   // end ctor



cv::Mat_<byte> AdaptiveDepthStructureFilter::filter( float minRng, float maxRng)
{ 
    _outImg = cv::Mat_<byte>::zeros( _rngImg.size());
    AdaptiveDepthPatchScanner adps( _rngImg, _rpatchSz, this);
    adps.scan( minRng, maxRng);
    return _outImg;
}   // end filter



void AdaptiveDepthStructureFilter::process( const cv::Point& p, float pdepth, const cv::Rect& patchRct)
{
    const cv::Mat_<float> rngMap = _rngImg;

    const int rowMax = patchRct.y + patchRct.height;
    const int colMax = patchRct.x + patchRct.width;

    list<int> dsmooth(1,0);
    list<int> dcounts(1,0);
    list<float> dmeans(1,0);

    list<int>::iterator dcIt = dcounts.begin();
    list<int>::iterator dsIt = dsmooth.begin();
    list<float>::iterator dmIt = dmeans.begin();

    int totalSmooth = 0;
    int totalCount = 0;
    int maxCommon = 0;
    int maxSmooth = 0;

    const float G = 0.1f/(patchRct.width * patchRct.height);
    float lastDepth = -1;

    for ( int i = patchRct.y; i < rowMax; ++i)
    {
        if ( i < 0)
           continue;
        else if ( i >= rngMap.rows)
            break;

        bool goRight = i % 2 == 0;
        int j = goRight ? patchRct.x : colMax-1;

        for ( int z = 0; z < patchRct.width; ++z)   // z is simply a count
        {
            if ( goRight)
            {
                if ( j < 0)
                    continue;
                else if ( j >= rngMap.cols)
                    break;
            }   // end if
            else
            {
                if ( j < 0)
                    break;
                else if ( j >= rngMap.cols)
                    continue;
            }   // end else

            const float d = _rngImg.at<float>(i,j);  // Depth at this point
            if ( d > 0) // Only if depth is present
            {
                if ( lastDepth > 0)
                {
                    const float ddiff = fabs(d-lastDepth);
                    if (ddiff < G)
                    {
                        (*dsIt)++;
                        totalSmooth++;
                        if (*dsIt > maxSmooth)
                            maxSmooth = *dsIt;
                    }   // end if
                    else if ( d > lastDepth)    // This point is further away
                    {
                        // Find the set of depth points closest to this one
                        while ( (dmIt != dmeans.end()) && (*dmIt < d) && fabs( *dmIt - d) >= G)
                        {
                            dmIt++; dcIt++; dsIt++;
                        }   // end while

                        // Check if we need to insert a new entry
                        if ( dmIt == dmeans.end() || fabs(d - *dmIt) >= G)
                        {
                            dsIt = dsmooth.insert( dsIt,0);
                            dmIt = dmeans.insert( dmIt,0);
                            dcIt = dcounts.insert( dcIt,0);
                        }   // end if
                    }   // end else if
                    else // This point is closer in
                    {
                        // Find the set of depth points closest to this one
                        while ( (dmIt != dmeans.begin()) && (*dmIt > d) && fabs( *dmIt - d) >= G)
                        {
                            dmIt--; dcIt--; dsIt--;
                        }   // end while

                        // Check if we need to insert a new entry
                        if ( fabs(d - *dmIt) >= G)
                        {
                            dmIt++; dcIt++; dsIt++;
                            dsIt = dsmooth.insert( dsIt,0);
                            dmIt = dmeans.insert( dmIt,0);
                            dcIt = dcounts.insert( dcIt,0);
                        }   // end if
                    }   // end else
                }   // end if

                *dmIt = (*dmIt * *dcIt + d) / (*dcIt + 1);
                (*dcIt)++;
                totalCount++;
                if ( *dcIt > maxCommon)
                    maxCommon = *dcIt;

                lastDepth = d;
            }   // end if

            j += goRight ? 1 : -1;
        }   // end for
    }   // end for

    //const int N = (int)dcounts.size();
    int pval = int(double(totalSmooth) / (totalCount) * 255.);

    maxSmooth += 1;
    totalSmooth += 1;
    //pval = double(maxCommon)/totalCount * double(totalSmooth)/totalCount * 255;
    //pval = double(maxCommon)/(totalSmooth) * 255;
    pval = int(double(maxCommon)/(totalCount) * 255.);

    _outImg.at<byte>(p) = pval;
}   // end process



