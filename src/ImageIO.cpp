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

#include <ImageIO.h>
#include <iostream>
#include <cassert>
#include <cstring>
#include <cstdio>
typedef unsigned char byte;


struct TGAHeader
{
    byte idlength;
    byte colourmaptype;
    byte datatypecode;

    short colourmaporigin;
    short colourmaplength;
    byte colourmapdepth;

    short x_origin;
    short y_origin;

    short width;
    short height;
    byte bitsperpixel;
    byte imagedescriptor;

    // Because of byte alignment, we can't trust that sizeof(TGAHeader) == 18
    byte barray[18];

    TGAHeader( const cv::Mat& m)
        : idlength(0), colourmaptype(0), datatypecode(2),   // Uncompressed RGB
          colourmaporigin(0), colourmaplength(0), colourmapdepth(0),
          x_origin(0), y_origin(0), height(m.rows), width(m.cols),
          bitsperpixel( m.channels() * 8), imagedescriptor(0)
    {
        memcpy( &barray[0], &idlength, 1);
        memcpy( &barray[1], &colourmaptype, 1);
        memcpy( &barray[2], &datatypecode, 1);
        memcpy( &barray[3], &colourmaporigin, 2);
        memcpy( &barray[5], &colourmaplength, 2);
        memcpy( &barray[7], &colourmapdepth, 1);
        memcpy( &barray[8], &x_origin, 2);
        memcpy( &barray[10], &y_origin, 2);
        memcpy( &barray[12], &width, 2);
        memcpy( &barray[14], &height, 2);
        memcpy( &barray[16], &bitsperpixel, 1);
        memcpy( &barray[17], &imagedescriptor, 1);
    }   // end ctor
};  // end struct


// public
bool RFeatures::saveAsTGA( const cv::Mat& m, const std::string& fname)
{
    if ( m.channels() < 3 || m.channels() > 4)
    {
        std::cerr << "[ERROR] RFeatures::saveAsTGA: only works with 3 or 4 channel images!" << std::endl;
        return false;
    }   // end if

    FILE *bstream = fopen( fname.c_str(), "w");
    if ( !bstream)
    {
        std::cerr << "Unable to open file for writing TGA image!" << std::endl;
        return false;
    }   // end if

    // Write the header
    TGAHeader tga(m);
    if ( fwrite( tga.barray, 1, 18, bstream) != 18)
    {
        std::cerr << "Failed to write TGA header!" << std::endl;
        return false;
    }   // end if

    // Write the image bytes row by row (BGA order)
    int bwrote = 0;
    const int nc = m.cols * m.channels();
    for ( int i = int(m.rows-1); i >= 0; --i)    // Write bottom to top
        bwrote += (int)fwrite( (void*)m.ptr(i), 1, nc, bstream);

    // Check all bytes written okay
    if ( bwrote != int(nc * m.rows))
    {
        std::cerr << "Failed to write all " << (nc * m.rows) << " bytes of the image!" << std::endl;
        return false;
    }   // end if

    fclose(bstream);    // flush & close
    return true;
}   // end saveAsTGA
