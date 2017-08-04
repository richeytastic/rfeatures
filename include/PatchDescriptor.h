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

#ifndef RFEATURES_PATCH_DESCRIPTOR_H
#define RFEATURES_PATCH_DESCRIPTOR_H

#ifdef _WIN32
#pragma warning( disable : 4251)
#endif

#include <vector>
using std::vector;
#include <list>
using std::list;
#include <string>
using std::string;
#include <iostream>
using std::ostream;
using std::istream;
#include <opencv2/opencv.hpp>
#include <boost/shared_ptr.hpp>
#include "rFeatures_Export.h"


namespace RFeatures
{

// Stores feature vectors describing an image patch along with a vector giving the offset
// from a patch to an object's reference point
class rFeatures_EXPORT PatchDescriptor
{
public:
    typedef boost::shared_ptr<PatchDescriptor> Ptr;

    // Load a bunch of new PatchDescriptor objects from file - returns number loaded.
    static int load( const string& fname, vector<PatchDescriptor::Ptr>& pfs);
    static PatchDescriptor::Ptr create() { return PatchDescriptor::Ptr( new PatchDescriptor);}

    // Converts the given single row or column matrix into a PatchDescriptor object
    // by taking the first two values and setting these to be the x ([0]) and y ([1])
    // offset values and the remaining values to be the feature vectors.
    PatchDescriptor( const cv::Mat_<float> singleRowOrColumn);

    explicit PatchDescriptor( const cv::Vec2f offset); // Add features later
    PatchDescriptor(){}

    // Set and get the offset (metrics undefined)
    void setOffset( const cv::Vec2f& offset);
    const cv::Vec2f& getOffset() const { return _offset;}

    int getNumFeatureVectors() const { return (int)_featureVecs.size();}

    // Adds the given feature vector and returns the total number added so far.
    // Can accept rectangular feature vectors (multi column and row).
    int addFeatureVector( const cv::Mat_<float> fv);
    int addRowFeatureVectors( const cv::Mat_<float> fv);    // Add a bunch as row vectors of equal length
    int addFeatureVectors( const list<cv::Mat_<float> >& fvs);    // Add a bunch

    // More flexible - add single feature vectors as arrays with given start and end pointers (endPtr is one PAST the end).
    int addFeatureVector( const float* startPtr, const float* endPtr);

    const cv::Mat_<float>& getFeatureVector( int i=0) const { return _featureVecs[i];}

    int getFeatureVectorSize( int i) const { return (int)_featureVecs[i].total();}

    // Get the value of feature vector fv at index i (assumes the feature vector is unit row or column).
    float val( int fv, int i) const;

private:
    cv::Vec2f _offset;  // Width, height offset patch to object reference point

    // featureVecs: row vectors (represented as cv::Mat_<float>) representing the features describing this patch.
    // Different feature types should have a different index in descriptors. In the normal case, as given
    // by the first two ctors, the descriptor is a single feature vector set in the first position of this array.
    vector< cv::Mat_<float> > _featureVecs;

    friend istream& operator>>( istream& is, PatchDescriptor& pf);
    bool importFormatted( const cv::Mat_<float> pf);
};  // end struct


ostream& operator<<( ostream& os, const PatchDescriptor& pf);
istream& operator>>( istream& is, PatchDescriptor& pf);

}   // end namespace

#endif

