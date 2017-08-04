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

#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <cmath>
#include <ctime>
#include <algorithm>
#include <vector>
#include <iomanip>
#include "../../include/NOctree.h"
using RFeatures::NOctree;

double dist( const float p[2], const float f[2])
{
    return pow(p[0] - f[0], 2) + pow(p[1] - f[1], 2);
}   // end dist


struct Sorter
{
    Sorter( const float *f) : _f(f) {}

    bool operator()( const float* p0, const float* p1)
    {
        return dist( p0, _f) >= dist( p1, _f);
    }   // end sorter

    const float *_f;
};  // end struct


void setPoints( std::vector<float*>& pts, int M, const float* f)
{
    srandom( (2*f[0]+1) * f[1]);
    pts.resize(M);
    for ( int i = 0; i < M; ++i)
    {
        pts[i] = new float[2];
        pts[i][0] = 10 * (double(random() - RAND_MAX/2) / (RAND_MAX/2));
        pts[i][1] = 10 * (double(random() - RAND_MAX/2) / (RAND_MAX/2));
    }   // end for

    Sorter sorter(f);
    std::sort( pts.begin(), pts.end(), sorter);

    for ( int i = 0; i < M; ++i)
    {
        const float *p = pts[i];
        std::cerr << "Inserting point " << std::setprecision(2) << std::fixed << std::right << std::setw(5) << p[0]
            << ", " << std::right << std::setw(5) << p[1] << " -- distance to search point = "
            << std::fixed << std::setw(8) << std::right << dist(p,f) << std::endl;
    }   // end for
}   // end setPoints



int main( int argc, char **argv)
{
    if ( argc != 3)
    {
        std::cerr << "Usage: " << argv[0] << " x y" << std::endl;
        return EXIT_FAILURE;
    }   // end if

    float f[2];
    sscanf( argv[1], "%f", &f[0]);
    sscanf( argv[2], "%f", &f[1]);

    const int M = 6;
    std::vector<float*> pts;
    setPoints( pts, M, f);

    std::cout << "Placing 2D points into NOctree" << std::endl;
    NOctree<float,2> *octree = new NOctree<float, 2>( pts[0]);
    for ( int i = 1; i < M; ++i)
        octree->insert( pts[i]);

    /*
    NOctree<float,2> *octree = new NOctree<float, 2>( p);
    std::cerr << "Inserting point " << p[0] << ", " << p[1] << " -- distance to search point = " << dist(p,f) << std::endl;
    for ( int i = -1; i < 3; ++i)
    {
        p[0] = i;
        for ( int j = -1; j < 3; ++j)
        {
            p[1] = j;
            if ( p[0] == 0 && p[1] == 0)    // Already inserted
                continue;

            std::cerr << "Inserting point " << p[0] << ", " << p[1] << " -- distance to search point = " << dist(p,f) << std::endl;
            octree->insert( p);
        }   // end for
    }   // end for
    */

    std::cerr << "Looking for " << f[0] << ", " << f[1] << std::endl;
    const float *fn = octree->findNearest( f);

    if ( fn)
        std::cerr << "Found closest point (" << fn[0] << ", " << fn[1] << ") with distance = " << dist(fn,f) << std::endl;
    else
        std::cerr << "No closest point found." << std::endl;

    delete octree;

    return EXIT_SUCCESS;
}   // end main
