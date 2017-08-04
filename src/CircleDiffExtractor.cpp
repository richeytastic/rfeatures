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

#include "CircleDiffExtractor.h"
using RFeatures::CircleDiffExtractor;
#include <sstream>
#include <cassert>

CircleDiffExtractor::CircleDiffExtractor( int nps, cv::Mat m)
    : FeatureExtractor(m.size()), _nps(nps), _cdiff( new CircleDiff( m, nps))
{}   // end ctor


CircleDiffExtractor::CircleDiffExtractor()
    : FeatureExtractor(), _nps(8), _cdiff(NULL)
{}   // end ctor


CircleDiffExtractor::~CircleDiffExtractor()
{
    if ( _cdiff != NULL)
        delete _cdiff;
}   // end dtor



void CircleDiffExtractor::getValidImageTypes( vector<ImageType>& vimgTypes) const
{
    vimgTypes.push_back(Depth);
    vimgTypes.push_back(Light);
}   // end getValidImageTypes



string CircleDiffExtractor::getParams() const
{
    std::ostringstream oss;
    oss << _nps;
    return oss.str();
}   // end getParams



FeatureExtractor::Ptr CircleDiffExtractor::createFromParams( const string& params) const
{
    int nps;
    try
    {
        std::istringstream iss(params);
        iss >> nps;
    }   // end try
    catch ( const std::exception&)
    {
        throw ExtractorTypeException( "Couldn't read params for CircleDiffExtractor from string: " + params);
    }   // end catch

    CircleDiffExtractor* fx = new CircleDiffExtractor;
    fx->_nps = nps;
    return FeatureExtractor::Ptr(fx);
}   // end createFromParams



FeatureExtractor::Ptr CircleDiffExtractor::initExtractor( const cv::Mat img) const
{
    return FeatureExtractor::Ptr( new CircleDiffExtractor( _nps, img));
}   // end initExtractor



cv::Mat_<float> CircleDiffExtractor::extractFV( const cv::Rect rct) const
{
    assert( _cdiff != NULL);
    const cv::Mat fv = (*_cdiff)( rct);
    return RFeatures::toRowVector(fv);
}   // end extractFV
