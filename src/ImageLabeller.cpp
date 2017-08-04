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

#include "ImageLabeller.h"
using RFeatures::ImageLabeller;
#include <iostream>
#include <cassert>
#include <algorithm>


ImageLabeller::ImageLabeller( const cv::Mat_<byte>& bimg, byte fgVal) : _bimg(bimg), _fgVal(fgVal)
{
}   // end ctor



int ImageLabeller::operator()()
{
    assert( _bimg.isContinuous());
    const int rows = _bimg.rows;
    const int cols = _bimg.cols;

    const byte fgVal = _fgVal;

    cv::Mat_<int> labelIdxMap = cv::Mat_<int>::zeros( rows, cols);
    std::vector<int> labels;
    labels.push_back(0);

    int* abvLabRow = labelIdxMap.ptr<int>(0);

    for ( int i = 0; i < rows; ++i)
    {
        const byte* brow = _bimg.ptr<byte>(i);
        int *labRow = labelIdxMap.ptr<int>(i);

        int thisLabelIdx = 0;

        for ( int j = 0; j < cols; ++j)
        {
            if ( brow[j] == fgVal)
            {
                // Check for existing labels in the diagonals in the row above
                const int dlabVal0 = j > 0 ? labels[abvLabRow[j-1]] : 0;  // Above left
                const int dlabVal1 = j < cols - 1 ? labels[abvLabRow[j+1]] : 0;   // Above right

                if ( dlabVal0 == dlabVal1)
                {
                    if ( !thisLabelIdx)
                    {
                        if ( dlabVal0)
                            thisLabelIdx = dlabVal0;
                        else
                        {
                            thisLabelIdx = labRow[j];
                            if ( !thisLabelIdx)
                            {
                                // If no existing labels in upper diagonals, create a new label
                                labels.resize( labels.size() + 1);
                                thisLabelIdx = int(labels.size()) - 1;
                                labels[thisLabelIdx] = thisLabelIdx;    // Label value same as index
                            }   // end if
                        }   // end else
                    }   // end if
                }   // end if
                else if ( !dlabVal0)    // Got label at upper right (no label at upper left)
                {
                    int lv = dlabVal1;
                    if ( thisLabelIdx > 0)
                    {
                        //labels[thisLabelIdx] = dlabVal1;
                        const int hv = std::max<int>( thisLabelIdx, dlabVal1);
                        lv = std::min<int>( thisLabelIdx, dlabVal1);
                        labels[hv] = lv;
                    }   // end if

                    thisLabelIdx = lv;
                }   // end else if
                else if ( !dlabVal1)    // Got label at upper left (no label at upper right)
                    thisLabelIdx = dlabVal0;    // thisLabelIdx cannot logically be smaller (and not zero)
                else
                {   // Both not zero and unequal, so use the lower of the two, and set the other accordingly
                    if ( dlabVal0 < dlabVal1)
                    {
                        thisLabelIdx = dlabVal0;
                        labels[dlabVal1] = dlabVal0;
                    }   // end if
                    else
                    {
                        thisLabelIdx = dlabVal1;
                        labels[dlabVal0] = dlabVal1;
                    }   // end else
                }   // end else

                labRow[j] = thisLabelIdx; // Finally, set the label

                // Does the row below pixel j have an object pixel? If so, copy the index down
                if ( i < rows-1 && brow[j+cols] == fgVal)
                    labRow[j+cols] = thisLabelIdx;
            }   // end if
            else
                thisLabelIdx = 0;
        }   // end for - cols

        abvLabRow = labRow;
    }   // end for - rows

    // Collect the regions
    _regions.clear();
    boost::unordered_map<int,int> objMap;   // Object labels to object index

    cv::Mat_<int> nlabelMap = cv::Mat_<int>::zeros( rows, cols);

    //int wcount = 0;
    for ( int i = 0; i < rows; ++i)
    {
        const int* labRow = labelIdxMap.ptr<int>(i);
        for ( int j = 0; j < cols; ++j)
        {
            int objLabel = labels[labRow[j]];
            if ( !objLabel) // background is skipped
                continue;

            while ( labels[objLabel] != objLabel)
            {
                objLabel = labels[objLabel];
                //wcount++;
            }   // end while

            labels[labRow[j]] = objLabel;

            if ( !objMap.count(objLabel))
            {
                objMap[objLabel] = (int)_regions.size();
                _regions.resize( _regions.size() + 1);
            }   // end if

            const int objIdx = objMap[objLabel];
            _regions[objIdx].push_back( cv::Point(j,i));

            nlabelMap.at<int>(i,j) = objIdx+1;    // DEBUG
        }   // end for - cols
    }   // end for - rows

    //std::cerr << "wcount = " << wcount << std::endl;
    nlabelMap.convertTo( _labImg, CV_8U, 255./_regions.size());

    return getNumRegions();
}   // end operator()



int ImageLabeller::getNumRegions() const
{
    return (int)_regions.size();
}   // end getNumRegions



const std::vector<cv::Point>* ImageLabeller::getRegionPoints( int fgRegion) const
{
    if ( fgRegion < 0 || fgRegion >= getNumRegions())
        return NULL;

    return &_regions[fgRegion];
}   // end getRegionPoints



int ImageLabeller::getRegionSizes( std::vector<int>& regSizes) const
{
    regSizes.clear();
    const int nregs = getNumRegions();
    regSizes.resize( nregs);
    for ( int i = 0; i < nregs; ++i)
        regSizes[i] = (int)_regions[i].size();
    return nregs;
}   // end getRegionSizes



const std::vector<cv::Point>* ImageLabeller::getLargestRegion() const
{
    const int nregs = getNumRegions();
    if ( nregs == 0)
        return NULL;
    const std::vector<cv::Point>* reg = &_regions[0];
    for ( int i = 1; i < nregs; ++i)
    {
        if ( _regions[i].size() > reg->size())
            reg = &_regions[i];
    }   // end for
    return reg;
}   // end getLargestRegion
