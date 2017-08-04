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

#include "LocalBinaryPatternExtractor.h"
using RFeatures::LocalBinaryPatternExtractor;
#include <cassert>


LocalBinaryPatternExtractor::LocalBinaryPatternExtractor( cv::Mat m)
    : FeatureExtractor(m.size()), _lbp( new LocalBinaryPattern(m))
{}   // end ctor


LocalBinaryPatternExtractor::LocalBinaryPatternExtractor()
    : FeatureExtractor(), _lbp(NULL)
{}   // end ctor


LocalBinaryPatternExtractor::~LocalBinaryPatternExtractor()
{
    if ( _lbp!= NULL)
        delete _lbp;
}   // end dtor


void LocalBinaryPatternExtractor::getValidImageTypes( vector<ImageType>& vimgTypes) const
{
    vimgTypes.push_back(Depth);
    vimgTypes.push_back(EDT);
    vimgTypes.push_back(Light);
    vimgTypes.push_back(Grey);
}   // end getValidImageTypes


FeatureExtractor::Ptr LocalBinaryPatternExtractor::createFromParams( const string& params) const
{
    LocalBinaryPatternExtractor *dd = new LocalBinaryPatternExtractor;
    return FeatureExtractor::Ptr(dd);
}   // end createFromParams



FeatureExtractor::Ptr LocalBinaryPatternExtractor::initExtractor( const cv::Mat img) const
{
    return FeatureExtractor::Ptr( new LocalBinaryPatternExtractor( img));
}   // end initExtractor



cv::Mat_<float> LocalBinaryPatternExtractor::extractFV( const cv::Rect rct) const
{
    // Cannot extract over rectangles with dimensions less than 3x3 pixels
    if ( rct.width < 3 || rct.height < 3)
        return cv::Mat_<float>();

    assert( _lbp!= NULL);
    const cv::Mat_<float> fv = (*_lbp)(rct);
    assert( fv.total() == 1);
    return RFeatures::toRowVector(fv);
}   // end extractFV
