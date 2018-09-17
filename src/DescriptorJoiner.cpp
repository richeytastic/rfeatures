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

#include <DescriptorJoiner.h>
using RFeatures::DescriptorJoiner;


cv::Mat_<float> DescriptorJoiner::loadDescriptors( const string& dfile, int* label) throw (DescriptorLengthException)
{
    const cv::Mat_<float> vecs = RFeatures::readDescriptors( dfile, false);
    const int numVecs = vecs.rows;
    const int lab = (int)_labCounts.size();  // Label for these desciptors
    _labCounts.push_back(numVecs);  // Store the number of descriptors for this class label (vector index)

    // Add vecs to _xs
    for ( int i = 0; i < numVecs; ++i)
    {
        _xs.push_back( vecs.row(i));
        _labs.push_back(lab);
    }   // end for

    return vecs.clone();
}   // end loadDescriptors


int DescriptorJoiner::getNumClasses() const
{
    return (int)_labCounts.size();
}   // end getNumClasses


int DescriptorJoiner::getDescriptorCount( int label) const
{
    if ( label < 0)
        return _xs.rows;
    else if ( label >= (int)_labCounts.size())
        return 0;
    return _labCounts[label];
}   // end getDescriptorCount


cv::Mat_<float> DescriptorJoiner::getRowDescriptors( int label) const
{
    cv::Mat_<float> vecs;

    if ( label < 0)
        return getAllRowDescriptors();
    else if ( label >= (int)_labCounts.size())
        return vecs;    // Empty

    // Find the start index into _xs
    int startIdx = 0;
    for ( int i = 0; i < label; ++i)
        startIdx += _labCounts[i];

    const int endIdx = startIdx + getDescriptorCount(label);
    for ( int i = startIdx; i < endIdx; ++i)
        vecs.push_back( _xs.row(i));

    return vecs.clone();
}   // end getRowDescriptors


cv::Mat_<float> DescriptorJoiner::getAllRowDescriptors() const
{
    return _xs.clone();
}   // end getAllRowDescriptors


cv::Mat_<int> DescriptorJoiner::getLabels() const
{
    return _labs.clone();
}   // end getLabels
