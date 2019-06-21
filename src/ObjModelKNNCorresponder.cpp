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

#include <ObjModelKNNCorresponder.h>
#include <iostream>
#include <cassert>
#include <cmath>
using RFeatures::ObjModelKNNCorresponder;
using RFeatures::ObjModelKDTree;
using RFeatures::ObjModel;


ObjModelKNNCorresponder::ObjModelKNNCorresponder( const ObjModel& mask) : _mask(mask)
{
    assert(mask.hasSequentialVertexIds());
}   // end ctor


cv::SparseMat_<float> ObjModelKNNCorresponder::sample( const ObjModelKDTree& kdt, int K) const
{
    K = std::max(1, K);
    const int n = static_cast<int>(_mask.numVtxs());   // # floating surface vertices
    const int m = int(kdt.numVtxs()); // # target vertices

    if ( K > m)
    {
        std::cerr << "[ERROR] RFeatures::ObjModelKNNCorresponder::sample: K exceeds number of target mesh points!" << std::endl;
        K = m;
    }   // end if

    static const float EPS = 0.000001f; // Required in the case of any distance == 0 to prevent div-by-zero

    std::vector<int> kverts(K);    // K closest vertices on the target model.
    std::vector<float> sqdis(K);   // Corresponding squared distances of each closest vertex to the search vertex

    const int dims[2] = {n,m};
    cv::SparseMat_<float> A( 2, dims);

    for ( int i = 0; i < n; ++i)
    {
        const cv::Vec3f& v = _mask.vtx(i);
        kdt.findn( v, kverts, &sqdis);

        // The mean of the K nearest neighbour vertices on the source model is found, with the
        // contribution of each vertex to the mean scaled as the inverse of its squared distance
        // from the target model's search vertex.
        //cv::Vec3f ci(0,0,0);    // The correspondence point to be found
        //float denom = 0;
        for ( int k = 0; k < K; ++k)
        {
            const int j = kverts.at(k);
            const float aij = powf( EPS + sqdis.at(k), -1); // Affinity weight as inverse distance
            A.ref( i, j) = aij;   // Set the affinity matrix entry
            //ci += aij * tmod->vtx(j);
            //denom += aij;
        }   // end for

        //ci *= 1.0f/denom;
    }   // end for

    return A;
}   // end sample
