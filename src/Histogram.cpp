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

#include "Histogram.h"
using RFeatures::Histogram;



void createCummulativeHistogram( const vector<int>& hist, vector<int>& chist)
{
    const int nhist = (int)hist.size();
    chist.clear();
    chist.resize(nhist);
    int c = 0;
    for ( int i = 0; i < nhist; ++i)
    {
        c += hist[i];   // Accumulate
        chist[i] = c;
    }   // end for
}   // end createCummulativeHistogram



Histogram::Histogram( const cv::Mat_<byte>& m)
{
    RFeatures::createHistogram( m, _hist);
    createCummulativeHistogram( _hist, _chist);
}   // end ctor



void Histogram::drawHistograms( const cv::Size& sz) const
{
    const cv::Mat_<cv::Vec3b> himg = RFeatures::drawHistogram( _hist, sz, false);
    const cv::Mat_<cv::Vec3b> cimg = RFeatures::drawHistogram( _chist, sz, false);
    RFeatures::showImage( himg, "HISTOGRAM", false);
    RFeatures::showImage( cimg, "C-HISTOGRAM", false);
}   // end drawHistogram
