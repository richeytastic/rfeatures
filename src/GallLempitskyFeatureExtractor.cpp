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

#include "GallLempitskyFeatureExtractor.h"
using RFeatures::GallLempitskyFeatureExtractor;
#include <cassert>

GallLempitskyFeatureExtractor::GallLempitskyFeatureExtractor( cv::Mat m)
    : FeatureExtractor(m.size()), _glf( new GallLempitskyFeature( m))
{}  // end ctor


GallLempitskyFeatureExtractor::GallLempitskyFeatureExtractor()
    : FeatureExtractor(), _glf(NULL)
{}  // end ctor


GallLempitskyFeatureExtractor::~GallLempitskyFeatureExtractor()
{
    if ( _glf != NULL)
        delete _glf;
}   // end dtor


string GallLempitskyFeatureExtractor::getParams() const
{
    return "";
}   // end getParams


void GallLempitskyFeatureExtractor::getValidImageTypes( vector<ImageType>& vimgTypes) const
{
    vimgTypes.push_back(BGR);
}   // end getValidImageTypes


FeatureExtractor::Ptr GallLempitskyFeatureExtractor::createFromParams( const string& params) const
{
    return FeatureExtractor::Ptr( new GallLempitskyFeatureExtractor);
}   // end createFromParams


// protected
FeatureExtractor::Ptr GallLempitskyFeatureExtractor::initExtractor( const cv::Mat img) const
{
    return FeatureExtractor::Ptr( new GallLempitskyFeatureExtractor( img));
}   // end initExtractor


cv::Mat_<float> GallLempitskyFeatureExtractor::extractFV( const cv::Rect rct) const
{
    assert( _glf != NULL);
    return (*_glf)( rct);
}   // end extractFV
