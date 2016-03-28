#include "PatchFeatureExtractor.h"
using RFeatures::DescriptorExtractor;
using RFeatures::PatchFeatureExtractor;
#include <cassert>
#include <cstdlib>
#include <iostream>
using std::cerr;
using std::endl;
#include <sstream>



cv::Mat_<cv::Point> createRandomPixelLookupTable( const cv::Mat_<byte>& mask)
{
    cv::Mat_<cv::Point> lookupTable;
    for ( int i = 0; i < mask.rows; ++i)
    {
        const byte* maskRow = mask.ptr<byte>(i);
        for ( int j = 0; j < mask.cols; ++j)
        {
            if ( maskRow[j])
                lookupTable.push_back(cv::Point(j,i));
        }   // end for
    }   // end for
    return lookupTable;
}   // end createRandomPixelLookupTable



PatchFeatureExtractor::PatchFeatureExtractor( const cv::Mat img, DescriptorExtractor* dx, const cv::Mat_<byte> mask)
    : _img(img), _dextractor(dx)
{
    assert( dx != 0);
    cv::Mat_<byte> msk = mask;
    if ( msk.empty())
        msk = cv::Mat_<byte>::ones(img.size()) * 255;

    assert( img.size() == msk.size());
    _lookupPxls = createRandomPixelLookupTable( msk);
}   // end ctor



void checkPatchSize( const cv::Size2f& patchSz)
{
    assert( patchSz.width > 0 && patchSz.height > 0 && patchSz.width < 1 && patchSz.height < 1);

    // if patchSz is greater than half the image size (in either direction), issue
    // a warning because it will take a while to possibly find patches that fit within
    // the image.
    if ( patchSz.width > 0.5 || patchSz.height > 0.5)
    {
        cerr << "WARNING (PatchFeatureExtractor): requested patch size if greater than 0.5,0.5 (width,height)" << endl;
        cerr << "Might these features be too coarse to be useful? Also, it may take time to find patches that" << endl;
        cerr << "fit within the given image." << endl;
    }   // end if
}   // end checkPatchSize



// private
cv::Mat_<float> PatchFeatureExtractor::extractFeature( const cv::Rect& patchRect, const cv::Point& pxl,
                                const cv::Vec2f& imgCentre, const cv::Size& realPatchSz, int featureLen)
{
    // Calculate the patch feature descriptor using the delegate implementation of extractPatchFeature
    const cv::Mat patchImg = _img(patchRect);   // Patch sub-region of the image
    cv::Mat_<float> feature = _dextractor->extractFeature( patchImg);   // Virtual call
    if ( feature.rows > 1)
        feature = feature.t();  // Transpose - delegate gave us a column vector

    if ( feature.total() != featureLen)
    {
        std::ostringstream oss;
        oss << "ERROR (PatchFeatureExtractor): DescriptorExtractor delegate is giving incorrect length descriptors!";
        throw DescriptorLengthException( oss.str());
    }   // end if

    // Calculate the corresponding offset vector of the image centre to the centre pixel of the patch
    const cv::Vec2f pxlDiff( float(pxl.x) - imgCentre[0], float(pxl.y) - imgCentre[1]);   // Pixel difference
    const cv::Vec2f patchOffset( pxlDiff[0]/realPatchSz.width, pxlDiff[1]/realPatchSz.height);   // Relative patch offset
    _offsets.push_back(patchOffset);

    return feature;
}   // end extractFeature



cv::Mat_<float> PatchFeatureExtractor::extractRandomPatches( const cv::Size2f& patchSz, int count)
                                                                            throw (DescriptorLengthException)
{
    checkPatchSize( patchSz);
    const int featureLen = _dextractor->getFeatureLength();
    cv::Mat_<float> features;
    _offsets = cv::Mat_<cv::Vec2f>();   // Clear from previous call

    const cv::Rect imgRect(0,0,_img.cols, _img.rows);
    const cv::Vec2f imgCentre( float(_img.cols)/2, float(_img.rows)/2); // Centre point of image (object)

    const cv::Size realPatchSz( patchSz.width * float(_img.cols) + 0.5, patchSz.height * float(_img.rows) + 0.5);

    const int halfPatchCols = realPatchSz.width/2;
    const int halfPatchRows = realPatchSz.height/2;

    const double lookupTableLen = _lookupPxls.rows - 1;
    for ( int i = 0; i < count; ++i)    // Get count features
    {
        const cv::Point* pxl = 0;   // Will be centre pixel of patch
        cv::Rect patchRect;

        do
        {
            const int ridx = int((double(random())/RAND_MAX) * lookupTableLen + 0.5);
            pxl = _lookupPxls.ptr<cv::Point>(ridx);
            // Create the rectangle at the appropriate location and with patch dimensions
            patchRect = cv::Rect(pxl->x - halfPatchCols, pxl->y - halfPatchRows, realPatchSz.width, realPatchSz.height);
            // If intersection of patch rectangle and image rectangle is not exactly the patch rectangle, try again..
        } while ( (imgRect & patchRect) != patchRect);

        const cv::Mat_<float> patchFeature = extractFeature( patchRect, *pxl, imgCentre, realPatchSz, featureLen);
        features.push_back( patchFeature);
    }   // end for

    return features;
}   // end extractRandomPatches



cv::Mat_<float> PatchFeatureExtractor::extractPatches( const cv::Size2f& patchSz, const cv::Mat_<cv::Vec2f>& points)
                                                            throw (ImageOutOfBoundsException, DescriptorLengthException)
{
    checkPatchSize( patchSz);
    const int featureLen = _dextractor->getFeatureLength();
    cv::Mat_<float> features;
    _offsets = cv::Mat_<cv::Vec2f>();   // Clear from previous call

    const cv::Rect imgRect(0,0,_img.cols, _img.rows);
    const cv::Vec2f imgCentre( float(_img.cols)/2, float(_img.rows)/2); // Centre point of image (object)

    const cv::Size realPatchSz( patchSz.width * float(_img.cols) + 0.5, patchSz.height * float(_img.rows) + 0.5);
    const int halfPatchCols = realPatchSz.width/2;
    const int halfPatchRows = realPatchSz.height/2;

    assert( points.isContinuous());
    const int numPoints = points.total();
    const cv::Vec2f* pointsVec = points.ptr<cv::Vec2f>(0);
    for ( int i = 0; i < numPoints; ++i)
    {
        const cv::Vec2f& point = pointsVec[i];
        const cv::Point pxl( int(point[0] * float(imgRect.width - 1) + 0.5), int(point[1] * float(imgRect.height - 1) + 0.5));
        // Create the rectangle at the appropriate location and with patch dimensions
        const cv::Rect patchRect(pxl.x - halfPatchCols, pxl.y - halfPatchRows, realPatchSz.width, realPatchSz.height);
        // If this patch rectangle is not contained by the image rectangle, we have a user error
        if ( (imgRect & patchRect) != patchRect)
        {
            std::ostringstream oss;
            oss << "ERROR (PatchFeatureExtractor): Calculated patch dimensions lie outside of image dimensions" << endl
                << "for the given points." << endl
                << "Calculated patch rectangle: " << patchRect << endl
                << "Image rectangle: " << imgRect;
            throw ImageOutOfBoundsException( oss.str());
        }   // end if

        const cv::Mat_<float> patchFeature = extractFeature( patchRect, pxl, imgCentre, realPatchSz, featureLen);
        features.push_back( patchFeature);
    }   // end foreach

    return features;
}   // end extractPatches



cv::Mat_<cv::Vec2f> PatchFeatureExtractor::getRandomPatchOffsets() const
{
    return _offsets;
}   // end getRandomPatchOffsets



void PatchFeatureExtractor::extractRandomPatchFeatures( const cv::Size2f& patchSz, vector<PatchFeature>& pfs, int count)
{
    cv::Mat_<float> features = extractRandomPatches( patchSz, count);
    const int rows = features.rows;
    assert( rows == _offsets.rows);

    for ( int i = 0; i < rows; ++i)
    {
        PatchFeature pf;
        features.row(i).copyTo( pf.descriptor);
        pf.offset = *_offsets.ptr<cv::Vec2f>(i);
        pfs.push_back( pf);
    }   // end for
}   // end extractRandomPatchFeatures



void DescriptorExtractor::generatePatchFeatures( const vector<Instance>& instances, int numPatches,
                                const cv::Size2f& realPatchSize, vector<PatchFeature>& patchFeatures)
{
    BOOST_FOREACH ( const Instance& instance, instances)
    {
        cv::Mat_<byte> mask;
        const cv::Mat target = this->getFeatureData( instance, realPatchSize, mask);
        assert( mask.empty() || mask.size() == target.size());
        PatchFeatureExtractor patchExtractor( target, this, mask);
        patchExtractor.extractRandomPatchFeatures( realPatchSize, patchFeatures, numPatches);
    }   // end foreach
}   // end generatePatchFeatures
