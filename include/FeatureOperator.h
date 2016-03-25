#pragma once
#ifndef RFEATURES_FEATURE_OPERATOR_H
#define RFEATURES_FEATURE_OPERATOR_H

#include "RFeatures.h"
#include "FeatureUtils.h"
#include "FeatureExceptions.h"
using RFeatures::FeatureSizeException;
#include <string>
using std::string;
#include <vector>
using std::vector;


namespace RFeatures
{

class rFeatures_EXPORT FeatureOperator
{
public:
    // Provide the size of the whole image features are to be extracted from, along
    // with an (optional) fixed feature size. If left at default (no size specified),
    // extracted feature vectors may or may not be variable in size.
    // Feature sampling method is to discretely sample the processed image
    // over the required arbitrary rectangular area. Features are however sampled
    // in proportion to the width and height of the rectangular area (which is why
    // a 2D size for the feature vector is provided).
    // THROWS FeatureSizeException if imgSz is too small for the fvDims.
    // (fvDims.width <= imgSz.width && fvDims.height <= imgSz.height must be true)
    FeatureOperator( const cv::Size& imgSz, const cv::Size fvDims=cv::Size(0,0)) throw (FeatureSizeException);
    virtual ~FeatureOperator(){}

    cv::Mat_<float> operator()() const;   // Extract from whole "image"

    // Extract from sub-region. Checks dimensions of sub-region first to ensure
    // sub-region is within image bounds.
    cv::Mat_<float> operator()( const cv::Rect&) const;

protected:
    // Extract features from the given rectangular area of the image.
    // Parameter rectangle is guaranteed to always be within srcDims_
    // as long as called via one of the operator functions above.
    // If the returned matrix is 2D (has more than 1 column and more than 1 row),
    // then its rows must equal the number of different features.
    // The length of the returned feature must be equal to some scaling
    // of the parameter rectangle.
    virtual cv::Mat_<float> extract( const cv::Rect&) const;

    // If client is not overriding extract, or provides a fixed size feature dimension
    // in the constructor (fvDims), this function MUST be overridden to supply the
    // sample image(s) from which to extract the feature vector(s).
    // All provided cv::Mat must be single channelled and of the same depth (e.g. CV_32F)
    virtual void getSampleChannels( const cv::Rect&, vector<cv::Mat>& simgs) const;

private:
    const cv::Rect _srcDims;    // Image dimensions
    const cv::Size _fvDims;     // Fixed feature dimensions (if used)

    // Returned matrix has N rows of feature vectors.
    cv::Mat_<float> discretelySample( const cv::Rect& rct, const cv::Size& fvDims) const;
};  // end class



/*
// Discretely sample the given patch samples into a standard (fvDims) size.
template <typename T>
cv::Mat_<float> dsample( const vector<cv::Mat>& simgs, const cv::Size& fvDims)
{
    const cv::Size imgSz = simgs[0].size();
    const int nrows = fvDims.height;
    const int ncols = fvDims.width;
    const double rowChunk = double(imgSz.height) / nrows;
    const double colChunk = double(imgSz.width) / ncols;
    const double hrowChunk = rowChunk/2;
    const double hcolChunk = colChunk/2;
    const int nvecs = simgs.size();

    // Returned matrix has same number of row vectors as input sample images
    cv::Mat_<float> fv( nvecs, nrows * ncols);

    double idx = -hrowChunk;    // Input row base index
    for ( int i = 0; i < nrows; ++i)
    {
        idx += rowChunk;
        // Pointers to the correct row of each input image.
        const T** srows = (const T**)cv::fastMalloc(nvecs * sizeof(T*));

        for ( int k = 0; k < nvecs; ++k)
            srows[k] = simgs[k].ptr<T>(int(idx));

        double jdx = -hcolChunk;    // Input col base index
        const int odx = i*ncols;    // Output base index
        for ( int j = 0; j < ncols; ++j)
        {
            jdx += colChunk; // Input index
            // Get values from the correct index into each row of the sample images
            for ( int k = 0; k < nvecs; ++k)
                fv.ptr<float>(k)[odx + j] = srows[k][int(jdx)];
        }   // end for - width

        cv::fastFree(srows);
    }   // end for - height
    return fv;
}   // end dsample
*/


}   // end namespace

#endif
