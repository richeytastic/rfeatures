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

#include <Orientation.h>
#include <Transformer.h>
using RFeatures::Orientation;
#include <boost/property_tree/xml_parser.hpp>


Orientation::Orientation()
    : _nvec(0,0,1), _uvec(0,1,0)
{
}   // end ctor


Orientation::Orientation( const PTree& ptree)
{
    ptree >> *this;
}   // end ctor


Orientation::Orientation( const cv::Vec3f& nv, const cv::Vec3f& uv)
    : _nvec(nv), _uvec(uv)
{
}   // end ctor


void Orientation::rotate( const cv::Matx44d& T)
{
    const RFeatures::Transformer mover(T);    // Don't translate!
    mover.rotate( _nvec);
    mover.rotate( _uvec);
}   // end rotate


void RFeatures::putVertex( PTree& node, const cv::Vec3f& v)
{
    node.put( "x", v[0]);
    node.put( "y", v[1]);
    node.put( "z", v[2]);
}   // end putVertex


void RFeatures::putNamedVertex( PTree& node, const std::string& label, const cv::Vec3f& v)
{
    putVertex( node.put( label, ""), v);
}   // end putNamedVertex


cv::Vec3f RFeatures::getVertex( const PTree& node)
{
    return cv::Vec3f( node.get<float>("x"), node.get<float>("y"), node.get<float>("z"));
}   // end getVertex


bool RFeatures::getNamedVertex( const PTree& n0, const std::string& label, cv::Vec3f& v)
{
    if ( n0.count(label) == 0)
        return false;
    v = getVertex( n0.get_child(label));
    return true;
}   // end getNamedVertex


// public
PTree& RFeatures::operator<<( PTree& record, const Orientation& v)
{
    PTree& orientation = record.put( "orientation", "");
    PTree& normal = orientation.add( "normal","");
    putVertex( normal, v.norm());
    PTree& upnode = orientation.add( "up","");
    putVertex( upnode, v.up());
    return record;
}   // end operator<<


// public
const PTree& RFeatures::operator>>( const PTree& record, Orientation& v)
{
    const PTree& orientation = record.get_child( "orientation");
    v.norm() = getVertex( orientation.get_child("normal"));
    v.up() = getVertex( orientation.get_child("up"));
    return record;
}   // end operator>>


// public
std::ostream& RFeatures::operator<<( std::ostream& os, const Orientation& v)
{
    os << v.norm() << v.up();
    return os;
}   // end operator<<
