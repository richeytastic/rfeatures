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

#include <PointCloudTextReader.h>
using RFeatures::PointCloudTextReader;
#include <sstream>
#include <iostream>


PointCloudTextReader::PointCloudTextReader()
{}   // end ctor



void PointCloudTextReader::read( istream &is)
{
    string ln;
    size_t rows, cols;
    std::getline( is, ln);
    std::istringstream ciss(ln);
    ciss >> rows >> cols;

    const int numLines = rows * cols; // Number of lines to read in
    PointCloudReader::createPointCloud(rows,cols);

    size_t row, col;
    double x, y, z;
    byte r, g, b;

    for ( int i = 0; i < numLines; ++i)
    {
        if ( std::getline( is, ln) && !ln.empty())
        {
            std::istringstream iss( ln);
            iss >> row >> col >> x >> y >> z >> r >> g >> b;
            PointCloudReader::setPoint( row, col, x, y, z, r, g, b);
        }   // end if
        else
        {
            std::cerr << "ERROR: Unable to read in point text data from file!" << std::endl;
            is.setstate( std::ios::failbit);
            break;
        }   // end else
    }   // end while
}   // end read
