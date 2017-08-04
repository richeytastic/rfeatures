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

#include "Equi2Rect.h"
using RFeatures::Equi2Rect;
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265359
#endif

// public static
// From rectilinear coordinates (x,y), calculate the latitude and longitude on the spherical image.
void Equi2Rect::calc45LatLon( float x, float y, float centralLat, float centralLon, float& lat, float& lon)
{
    const float rho = sqrt(x*x + y*y);
    const float c = atanf( rho);
    const float sinfc = sinf(c);

    if ( fabs(rho) < 1e-8)
        lat = centralLat;
    else if ( fabs(centralLat) < 1e-8)
    {
        lat = asinf( y/rho * sinfc);// Simplified because centralLat = 0
        lon = atan2f( x*sinfc, rho*cosf(c));
    }   // end else if
    else
    {
        lat = asinf( cosf(c)*sinf(centralLat) + y/rho * sinfc *cosf(centralLat));
        lon = atan2f( x*sinfc, cosf(centralLat)* rho*cosf(c) - y*sinf(centralLat)*sinfc);
    }   // end else

    lon += centralLon;
}   // end calc45LatLon



// public
Equi2Rect::Equi2Rect( const cv::Mat& eqimg, float focalLen)
    : _eqimg( eqimg), _focalLen(focalLen)
{
}   // end ctor



// public
cv::Mat Equi2Rect::calcRectilinear( const cv::Size& dims, float xcentre, float ycentre) const
{
    xcentre *= _eqimg.cols;
    ycentre *= _eqimg.rows;

    const int rows = dims.height;
    const int cols = dims.width;
    const int hrows = rows/2;
    const int hcols = cols/2;

    static const float PIon4 = float(CV_PI/4);
    const float pxls45 = _focalLen*tanf(PIon4);   // Pxls wide or high in _eqimg that represent a 45 degree horizontal segment
    //std::cerr << "Focal length (pixels) = " << _focalLen << std::endl;
    //std::cerr << "pxls45 = " << pxls45 << std::endl;

    cv::Mat_<cv::Vec3b> outImg( dims);
    float lat, lon;

    for ( int i = -hrows; i < hrows; ++i)
    {
        const float y = float(i)/hrows; // From -1 to 1

        for ( int j = -hcols; j < hcols; ++j)
        {
            const float x = float(j)/hcols; // From -1 to 1

            Equi2Rect::calc45LatLon( x, y, 0, 0, lat, lon);
            // Convert lat and lon to pixel coords in spherical image
            const int sy = cvRound( lat/PIon4 * pxls45) - 1;
            const int sx = cvRound( lon/PIon4 * pxls45) % _eqimg.cols;
            outImg.at<cv::Vec3b>(hrows-i, hcols+j) = _eqimg.at<cv::Vec3b>( int(ycentre - sy), int(xcentre + sx));
        }   // end for - cols
    }   // end for - rows

    return outImg;
}   // end calcRectilinear


