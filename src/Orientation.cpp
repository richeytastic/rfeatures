/************************************************************************
 * Copyright (C) 2019 Richard Palmer
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
using RFeatures::Orientation;
#include <boost/property_tree/xml_parser.hpp>

Orientation::Orientation() : _nvec(0,0,1), _uvec(0,1,0) { }

Orientation::Orientation( const PTree& ptree) { ptree >> *this;}

Orientation::Orientation( const cv::Vec3f& n, const cv::Vec3f& u)
{
    setN(n);
    setU(u);
}   // end ctor

Orientation::Orientation( const cv::Matx44d& m)
{
    setN( cv::Vec3f( float(m(0,2)), float(m(1,2)), float(m(2,2))));
    setU( cv::Vec3f( float(m(0,1)), float(m(1,1)), float(m(2,1))));
}   // end ctor


void Orientation::rotate( const cv::Matx44d& T)
{
    rotate( cv::Matx33d( T(0,0), T(0,1), T(0,2),
                         T(1,0), T(1,1), T(1,2),
                         T(2,0), T(2,1), T(2,2)));
}   // end rotate


void Orientation::rotate( const cv::Matx33d& T)
{
    _nvec = T * _nvec;
    _uvec = T * _uvec;
}   // end rotate


cv::Matx44d Orientation::asMatrix( const cv::Vec3d& t) const
{
    cv::Vec3d xvec;
    cv::normalize( _uvec.cross(_nvec), xvec);
    return cv::Matx44d( xvec[0], _uvec[0], _nvec[0], t[0],
                        xvec[1], _uvec[1], _nvec[1], t[1],
                        xvec[2], _uvec[2], _nvec[2], t[2],
                            0.0,      0.0,      0.0,  1.0);
}   // end asMatrix


bool Orientation::operator==( const Orientation& o) const
{
    return _nvec == o._nvec && _uvec == o._uvec;
}   // end operator==


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
PTree& RFeatures::operator<<( PTree& orientation, const Orientation& v)
{
    PTree& normal = orientation.add( "normal","");
    putVertex( normal, v.nvec());
    PTree& upnode = orientation.add( "up","");
    putVertex( upnode, v.uvec());
    return orientation;
}   // end operator<<


// public
const PTree& RFeatures::operator>>( const PTree& orientation, Orientation& v)
{
    v.setN( getVertex( orientation.get_child("normal")));
    v.setU( getVertex( orientation.get_child("up")));
    return orientation;
}   // end operator>>


// public
std::ostream& RFeatures::operator<<( std::ostream& os, const Orientation& v)
{
    os << v.nvec() << v.uvec();
    return os;
}   // end operator<<
