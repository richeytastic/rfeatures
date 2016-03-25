#pragma once
#ifndef RFEATURES_FEATURE_EXTRACTOR2_H
#define RFEATURES_FEATURE_EXTRACTOR2_H
// Disable warnings about MSVC compiler not implementing exception specifications
#ifdef _WIN32
#pragma warning( disable : 4290)
#endif

#include <boost/shared_ptr.hpp>
#include <opencv2/opencv.hpp>
#include <string>
using std::string;
#include "FeatureUtils.h"
#include "View.h"
#include "ImageTypeEnum.h"
using RFeatures::ImageType;
#include "FeatureExceptions.h"
using RFeatures::ExtractorTypeException;


namespace RFeatures
{

// Extract feature vectors.
// The returned cv::Mat_<float> can have multiple rows if defining different feature types
// (one row per feature type) but the feature vectors must have the same length.
class rFeatures_EXPORT FeatureExtractor
{
public:
    typedef boost::shared_ptr<FeatureExtractor> Ptr;

    explicit FeatureExtractor( cv::Size imgSz=cv::Size(0,0));
    virtual ~FeatureExtractor();

    // Get the valid image types that this feature extractor can deal with.
    virtual void getValidImageTypes( vector<ImageType>&) const = 0;
    virtual string getTypeString() const = 0;   // E.g. Pro-HOG (defined by child)
    virtual string getParams() const = 0; // Get feature extraction parameters (defined by child)
    virtual cv::Size getFeatureDims() const = 0; // Gets the size of the feature as encoded

    // If this is set, all extracts from every extract function (including for pre-processed feature
    // extractors produced from this parent object), will first resize the specified subregion
    // (if using extract(cv::Rect)) or the extract itself (if using extract() or extract(cv::Mat)),
    // before performing actual feature extraction. This can be useful when comparing different
    // feature extractors when one wishes to control for extract dimensions.
    // Setting fixed dimensions may significantly slow down feature extractor processing.
    // Throws FeatureSizeException if fixedDims are smaller than the minimum sampling dimensions.
    void setFixedExtractSize( const cv::Size& fixedDims) throw (ImageSizeException);

    // Whether or not setFixedExtractDims has been set, extract from rct resized first it to the given dims.
    cv::Mat_<float> extract( const cv::Rect rct, const cv::Size& fixedDims) const throw (ImageSizeException);

    // Extract the feature vector over the given rectangular patch (for pre-processed feature extractors).
    cv::Mat_<float> extract( const cv::Rect rct) const throw (ImageSizeException);

    // Extract the feature vector for the whole image.
    cv::Mat_<float> extract() const;

    // Convenience function - will throw if ex is the wrong type of image for the feature extractor.
    cv::Mat_<float> extract( const cv::Mat& ex) const throw (ExtractorTypeException, ImageTypeException);


    // Do the necessary feature pre-processing on the given image or view, returning the feature extractor object.
    // The type of image provided in the first version of this function must match the already set image type!
    FeatureExtractor::Ptr preProcess( const cv::Mat img) const throw (ExtractorTypeException, ImageTypeException);
    FeatureExtractor::Ptr preProcess( const View::Ptr) const;

    // Gets the minimum size over which a feature can be extracted (e.g. 16x16 pixels)
    virtual cv::Size getMinSamplingDims() const;

    // Set & get the image type.
    void setImageType( ImageType);
    ImageType getImageType() const;
    const cv::Mat& getRawImage() const;
    const cv::Size& getFixedExtractSize() const;    // Returns cv::Size(0,0) if not fixed

    // Create a new feature extractor of the child type from the given parameters
    FeatureExtractor::Ptr createNew( const string& params) const throw (ExtractorTypeException);

    // Returns a string that can be used to construct this feature extractor type.
    string getConstructString() const;
    cv::Size getImageSize() const;

    bool isPreProcessed() const { return !_rawImg.empty();}

protected:
    cv::Size _imgSz;
    bool _imgTypeSet;
    ImageType _imgType;
    cv::Mat _rawImg;    // Image converted to the type required to process as _imgType
    cv::Size _fixedDims;

    // Child class creates a new instance of itself from the given params string.
    // If params are not suitable, child can return a NULL ptr or throw ExtractorTypeException.
    virtual FeatureExtractor::Ptr createFromParams( const string& params) const = 0;

    // Initialise the extractor against the image - error checking done here in parent class (preProcess).
    virtual FeatureExtractor::Ptr initExtractor( const cv::Mat img) const = 0;

    // Extract the feature vector for a rectangular patch into row vectors.
    // Each row can be a different feature, but the feature vectors must have the same length.
    // Returned vector may be empty if rct or the image type is invalid.
    virtual cv::Mat_<float> extractFV( const cv::Rect rct) const = 0;
};  // end class


}   // end namespace

#endif

