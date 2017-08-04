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

#include "EDTFeature.h"
using RFeatures::EDTFeature;


EDTFeature::EDTFeature( const cv::Mat_<byte> img, const cv::Size fvDims)
    : RFeatures::FeatureOperator( img.size(), fvDims)
{
    const cv::Mat_<int> sedt = RFeatures::DistanceTransform::calcdt( img);   // Distance map to edges
    cv::Mat fsedt;  // convert to 64 bit float for sqrt
    sedt.convertTo( fsedt, CV_64F);
    cv::sqrt( fsedt, _dtimg);
    cv::integral( _dtimg, _iimg, CV_64F);
}   // end ctor


cv::Mat_<float> EDTFeature::extract( const cv::Rect& rct) const
{
    cv::Mat_<double> dtpatch = _dtimg( rct);
    const double psum = RFeatures::getIntegralImageSum<double>( _iimg, rct); // Sum over the patch area
    cv::Mat_<float> fpatch( dtpatch.size());
    // Normalise patch values
    const int nrows = dtpatch.rows;
    const int ncols = dtpatch.cols;
    for ( int i = 0; i < nrows; ++i)
    {
        const double* dtrow = dtpatch.ptr<double>(i); // In row
        float* frow = fpatch.ptr<float>(i); // Out row
        for ( int j = 0; j < ncols; ++j)
            frow[j] = float(dtrow[j]/psum);
    }   // end for - rows
    return fpatch;
}   // end extract


void EDTFeature::getSampleChannels( const cv::Rect& rct, vector<cv::Mat>& simgs) const
{
    simgs.push_back( this->extract(rct));
}   // end getSampleChannels
