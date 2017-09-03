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

#include <ObjModelCropper.h>
#include <FeatureUtils.h>
using RFeatures::ObjModelBoundaryParser;
using RFeatures::ObjModelCropper;
using RFeatures::ObjModel;


// public
ObjModelCropper::ObjModelCropper( const cv::Vec3f& originVertex, double radiusThreshold)
        : _ov( originVertex), _sqRadiusThreshold(radiusThreshold*radiusThreshold)
{}  // end ctor


const std::list<int>& ObjModelCropper::getBoundary() const
{
    return _vboundaries.getBoundary(0);
}   // end getBoundary


bool ObjModelCropper::parseEdge( int fid, int v0, int v1)
{
    const bool parseEdge = l2sq( _ov - model->getVertex(v0)) <= _sqRadiusThreshold &&
                           l2sq( _ov - model->getVertex(v1)) <= _sqRadiusThreshold;
    if ( !parseEdge)
        _vboundaries.setEdge( v0, v1);
    return parseEdge;
}   // end parseEdge
