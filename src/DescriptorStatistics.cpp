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

#include "DescriptorStatistics.h"
using RFeatures::DescriptorStatistics;
#include "FeatureUtils.h"



int DescriptorStatistics::add( const cv::Mat& d) throw (DescriptorLengthException)
{
    const cv::Mat_<float> nd = RFeatures::modifyDescriptor( d, CV_32F);    // Single row vector
    if ( !_vecs.empty() && ( nd.cols != _vecs.cols))
        throw DescriptorLengthException("Descriptor length does not match existing descriptor lengths.");
    _vecs.push_back(nd);
    return _vecs.rows;
}   // end add


DescriptorStatistics& RFeatures::operator<<( DescriptorStatistics& ds, const cv::Mat& d)
{
    ds.add(d);
    return ds;
}   // end operator<<



ostream& RFeatures::operator<<( ostream& os, const DescriptorStatistics& ds)
{
    const int numArrs = ds._vecs.rows;
    if ( numArrs == 0)
        return os;

    const int numElems = ds._vecs.cols;
    os << "# descriptors = " << numArrs << std::endl;
    os << "# features per descriptor = " << numElems << std::endl;

    // Calc means (and print vectors)
    vector<float> means(numElems);
    for ( int i = 0; i < numArrs; ++i)
    {
        const float* fv = ds._vecs.ptr<float>(i);
        float vsum = 0;
        for ( int j = 0; j < numElems; ++j)
        {
            means[j] += fv[j] / numArrs;
            os << " " << std::fixed << std::setprecision(2) << std::setw(5) << fv[j];
            vsum += fv[j];
        }   // end for
        os << "  : sum = " << vsum << std::endl;
    }   // end for

    // Calc std-devs
    vector<float> diffs(numElems);
    for ( int i = 0; i < numArrs; ++i)
    {
        const float* fv = ds._vecs.ptr<float>(i);
        for ( int j = 0; j < numElems; ++j)
        {
            const float delta = means[j] - fv[j];
            diffs[j] += delta*delta;
        }   // end for
    }   // end for

    os << "Means:" << std::endl;
    for ( int i = 0; i < numElems; ++i)
        os << " " << std::fixed << std::setprecision(2) << std::setw(5) << means[i];
    os << std::endl;

    os << "Std-dev:" << std::endl;
    vector<float> stddevs(numElems);
    for ( int i = 0; i < numElems; ++i)
    {
        const float sdv = sqrt(diffs[i]/numArrs);
        stddevs[i] = sdv;
        os << " " << std::fixed << std::setprecision(2) << std::setw(5) << sdv;
    }   // end for
    os << std::endl;

    // Calculate proportion of entries < 1 stddev from mean
    vector<float> std1( numElems);
    vector<float> std2( numElems);
    vector<float> std3( numElems);
    float std1sum = 0;
    float std2sum = 0;
    float std3sum = 0;
    os << "Within 1 std-devs of mean:" << std::endl;
    for ( int i = 0; i < numElems; ++i)
    {
        // Mean and std-dev for entry i of the feature vector
        const float m = means[i];
        const float sdv = stddevs[i];

        for ( int j = 0; j < numArrs; ++j)
        {
            const float v = ds._vecs.at<float>(j,i);
            const float diff = fabs(v - m);
            if ( diff < sdv)
                std1[i] += 1.0f/numArrs;
            else if ( diff < 2*sdv)
                std2[i] += 1.0f/numArrs;
            else
                std3[i] += 1.0f/numArrs;
        }   // end for

        os << " " << std::fixed << std::setprecision(0) << std::setw(4) << (100*std1[i]) << "%";
        std1sum += std1[i];
        std2sum += std2[i];
        std3sum += std3[i];
    }   // end for
    os << "  : avg = " << (100*std1sum / numElems) << "%" << std::endl;

    os << "Within 2 std-devs of mean:" << std::endl;
    for ( int i = 0; i < numElems; ++i)
        os << " " << std::fixed << std::setprecision(0) << std::setw(4) << (100*std2[i]) << "%";
    os << "  : avg = " << (100*std2sum / numElems) << "%" << std::endl;

    os << "More than 2 std-devs from mean:" << std::endl;
    for ( int i = 0; i < numElems; ++i)
        os << " " << std::fixed << std::setprecision(0) << std::setw(4) << (100*std3[i]) << "%";
    os << "  : avg = " << (100*std3sum / numElems) << "%";

    return os;
}   // end operator<<



istream& RFeatures::operator>>( istream& is, DescriptorStatistics& ds)
{
    const bool asCols = false;
    while ( is.good())
    {
        const cv::Mat_<float> vec = RFeatures::readDescriptor( is, asCols);
        if ( !vec.empty())
            ds.add( vec);
    }   // end while
    return is;
}   // end operator>>
