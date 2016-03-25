#pragma once
#ifndef RFEATURES_DESCRIPTOR_JOINER_H
#define RFEATURES_DESCRIPTOR_JOINER_H

#include <string>
using std::string;
#include <vector>
using std::vector;
#include <opencv2/opencv.hpp>
#include "FeatureUtils.h"
#include "FeatureExceptions.h"
using RFeatures::DescriptorLengthException;


namespace RFeatures
{

class rFeatures_EXPORT DescriptorJoiner
{
public:
    // Loads descriptors from the given file (1 row per descriptor).
    // Returns the descriptors loaded from the file and if parameter
    // label is not null, sets this to be the label assigned to these
    // descriptors.
    cv::Mat_<float> loadDescriptors( const string& dfile, int* label=NULL) throw (DescriptorLengthException);

    // Returns the number of classes loaded (number of times loadDescriptors was called).
    int getNumClasses() const;

    int getDescriptorCount( int label=-1) const;

    // Return the descriptors with the given label or all descriptors
    // if label is set to -1 (default).
    cv::Mat_<float> getRowDescriptors( int label=-1) const;
    cv::Mat_<float> getAllRowDescriptors() const;   // Synonymous with getRowDescriptors(-1)

    // Returns all labels (as a continuous matrix) corresponding
    // to all row descriptors returned from a call to getRowDescriptors(-1).
    cv::Mat_<int> getLabels() const;

private:
    vector<int> _labCounts; // Number of instances per class
    cv::Mat_<float> _xs;
    cv::Mat_<int> _labs;
};  // end class

}   // end namespace

#endif
