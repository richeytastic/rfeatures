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

#include "PointCloud.h"
using RFeatures::PointCloud;
using RFeatures::PointXYZRGB;

const PointCloud::Ptr PointCloud::Null;


PointXYZRGB& PointCloud::packPoint( PointXYZRGB &pt,
                    double x, double y, double z, byte r, byte g, byte b)
{
    pt.x = (float)x;
    pt.y = (float)y;
    pt.z = (float)z;
    byte *rgb = (byte*)&pt.rgb;
    rgb[0] = r;
    rgb[1] = g;
    rgb[2] = b;
    return pt;
}   // end packPoint


void PointCloud::unpackPoint( const PointXYZRGB &pt,
                    float &x, float &y, float &z, byte &r, byte &g, byte &b)
{
    double dx, dy, dz;
    PointCloud::unpackPoint( pt, dx, dy, dz, r, g, b);
    x = (float)dx;
    y = (float)dy;
    z = (float)dz;
}   // end unpackPoint


void PointCloud::unpackPoint( const PointXYZRGB &pt,
                    double &x, double &y, double &z, byte &r, byte &g, byte &b)
{
    x = pt.x;
    y = pt.y;
    z = pt.z;
    byte *rgb = (byte*)&pt.rgb;
    r = rgb[0];
    g = rgb[1];
    b = rgb[2];
}   // end unpackPoint


#include "PointCloud_impl.h"
using RFeatures::PointCloud_impl;


PointCloud::Ptr PointCloud::create()
{
    return PointCloud::Ptr( new PointCloud_impl());
}   // end create


PointCloud::Ptr PointCloud::create( size_t width, size_t height)
{
    return PointCloud::Ptr( new PointCloud_impl( width, height));
}   // end create
