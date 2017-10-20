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
#include <cassert>
using RFeatures::ObjModelBoundaryParser;
using RFeatures::ObjModelCropper;
using RFeatures::ObjModel;


ObjModelCropper::ObjModelCropper( const cv::Vec3f& originVertex, double radiusThreshold)
    : _ov( originVertex),
    _sqRadiusThreshold(radiusThreshold*radiusThreshold),
    _vboundaries( new RFeatures::VertexBoundaries())
{}  // end ctor


ObjModelCropper::~ObjModelCropper()
{
    delete _vboundaries;
}   // end dtor


void ObjModelCropper::reset()
{
    delete _vboundaries;
    _vboundaries = new RFeatures::VertexBoundaries();
}   // end reset


const std::list<int>& ObjModelCropper::getBoundary() const
{
    return _vboundaries->getBoundary(0);
}   // end getBoundary


bool ObjModelCropper::parseEdge( int fid, int v0, int v1)
{
    assert( model->getFaceIds().count(fid) > 0);
    assert( model->getVertexIds().count(v0) > 0);
    assert( model->getVertexIds().count(v1) > 0);

    bool parseBoundary = true;
    const double f0 = l2sq( _ov - model->vtx(v0));
    const double f1 = l2sq( _ov - model->vtx(v1));

    /*
    if ( model->getNumSharedFaces(v0,v1) == 1 )
        std::cerr << "RFeatures::ObjModelCropper::parseEdge: adding model edge " << v0 << " --> " << v1 << std::endl;
    */
    if (( model->getNumSharedFaces(v0,v1) == 1 ) || (f0 >= _sqRadiusThreshold) && (f1 >= _sqRadiusThreshold))
    {
        _vboundaries->setEdge( v0, v1);
        parseBoundary = false;
    }   // end if
    return parseBoundary;
}   // end parseEdge


void ObjModelCropper::finishedParsing()
{
    _vboundaries->finish( model);
    assert( _vboundaries->getNumBoundaries() > 0);
    _vboundaries->sortBoundaries(true);
}   // end finishedParsing
