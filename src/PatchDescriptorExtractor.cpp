#include "PatchDescriptorExtractor.h"
using RFeatures::PatchDescriptorExtractor;
#include <iostream>
#include <algorithm>


// Set the absolute pixel location of the object reference point (e.g. the centre or bottom middle etc)
cv::Point calcOffsetAbsReference( const cv::Point2f& objRef, const cv::Rect& bbox)
{
    return cv::Point( bbox.x + cvRound(float(bbox.width) * objRef.x),
                      bbox.y + cvRound(float(bbox.height) * objRef.y));
}   // end calcOffsetAbsReference


void PatchDescriptorExtractor::createBoxSamplePoints()
{
    const int numPts = RFeatures::nonZeroToPoints( _mask(_bbox), _samplePts);
    if ( numPts == 0)
        throw RFeatures::ExtractorTypeException(
"[ERROR] PatchDescriptorExtractor::ctor: provided bounding box has no valid extraction locations according to given mask!");
}   // end createBoxSamplePoints



PatchDescriptorExtractor::PatchDescriptorExtractor(
        const cv::Mat_<float>& rng, const cv::Mat& mask, const cv::Rect& bbox,
        const cv::Point2f& oset, const cv::Size2f& realps, rlib::Random& rndNumGen)

    : _rng(rng), _mask(mask), _bbox(bbox), _realPatchSize(realps), _pxlPatchSize(0,0),
      _imgRect(0,0,rng.cols,rng.rows),
      _pRanger( new RFeatures::PatchRanger(rng)), _prect(0,0,0,0), _rndNumGen(rndNumGen),
      _minSamplingDims(1,1)
{
    _objRefPoint = calcOffsetAbsReference( oset, bbox);
    assert( mask.type() == CV_8UC1);
    createBoxSamplePoints();
}   // end ctor


PatchDescriptorExtractor::PatchDescriptorExtractor(
        const cv::Mat& mask, const cv::Rect& bbox,
        const cv::Point2f& oset, const cv::Size2i& pxlps, rlib::Random& rndNumGen)

    : _mask(mask), _bbox(bbox), _realPatchSize(0,0), _pxlPatchSize(pxlps),
      _imgRect(0,0,mask.cols,mask.rows),
      _pRanger(NULL), _prect(0,0,0,0), _rndNumGen(rndNumGen),
      _minSamplingDims(1,1)
{
    _objRefPoint = calcOffsetAbsReference( oset, bbox);
    assert( mask.type() == CV_8UC1);
    createBoxSamplePoints();
}   // end ctor


PatchDescriptorExtractor::~PatchDescriptorExtractor()
{
    if ( _pRanger)
        delete _pRanger;
}   // end dtor


// public
void PatchDescriptorExtractor::addFX( const FX& fx)
{
    // fx must be pre-processed on an input image already!
    if ( !fx->isPreProcessed())
        throw RFeatures::ExtractorTypeException( "[ERROR] PatchDescriptorExtractor::addFX: added FX not yet preprocessed!");
    _fxs.push_back(fx);
    const cv::Size msz = fx->getMinSamplingDims();
    _minSamplingDims.width = std::max<int>( _minSamplingDims.width, msz.width);
    _minSamplingDims.height = std::max<int>( _minSamplingDims.height, msz.height);
}   // end addFX


// public
const cv::Rect& PatchDescriptorExtractor::sampleRandomPatch( int maxtries)
{
    if ( _fxs.empty())
        throw RFeatures::ExtractorTypeException( "[ERROR] PatchDescriptorExtractor::sampleRandomPatch: No FXs added!");

    cv::Point pt;
    cv::Rect pRect = cv::Rect( -1, -1, 1, 1);
    _prect = cv::Rect(0,0,0,0);

    // Ensure the patch rectangle that is extracted is wholly in the image rectangle
    // and that it's at least as large as the minimum sampling dimensions for the FX
    // requiring the largest sampling dims.
    int tries = 0;
    while (true)
    {
        tries++;
        if ( tries > maxtries)  // FAIL!
            return _prect;

        // Find a random point within the instance's bounding box
        const int smpIdx = _rndNumGen.getRandomInt() % _samplePts.size();
        const cv::Point& spt = _samplePts[smpIdx];
        pt = cv::Point( _bbox.x + spt.x, _bbox.y + spt.y);

        // fixed or scaled size patch?
        if ( _pRanger)  // Scaled
            _pRanger->calcPatchRect( pt, _realPatchSize, pRect);
        else
        {   // Fixed
            pRect = cv::Rect( pt.x - int(_pxlPatchSize.width/2),
                              pt.y - int(_pxlPatchSize.height/2), _pxlPatchSize.width, _pxlPatchSize.height);
        }   // end else

        if ( ( _imgRect & pRect) != pRect)
            continue;
       
        if ( pRect.width < _minSamplingDims.width || pRect.height < _minSamplingDims.height)
            continue;

        break;
    }   // end while

    _prect = pRect;
    _absPt = pt;
    return _prect;
}   // end sampleRandomPatch


// public
PatchDescriptor::Ptr PatchDescriptorExtractor::extractPatch() const
{
    // Offset to object reference point from patch always as a proportion of the patch dimensions
    const cv::Vec2f offset = cv::Vec2f( float(_objRefPoint.x - _absPt.x) / _prect.width,
                                        float(_objRefPoint.y - _absPt.y) / _prect.height);
    PatchDescriptor::Ptr pd( new PatchDescriptor( offset));

    // Debug
    if ( !_timg.empty())
    {
        cv::rectangle( _timg, _prect, cv::Scalar(255,255,0), 1);
        cv::line( _timg, _absPt, _objRefPoint, cv::Scalar(200,200,0), 1);
        std::cout << "Offset = " << offset[0] << ", " << offset[1] << std::endl;
        RFeatures::showImage( _timg, "Patch Extracts", true);
    }   // end if

    // Extract the feature vectors for this patch and add to the patch descriptor
    const int numfxs = _fxs.size();
    for ( int j = 0; j < numfxs; ++j)
    {
        // Extract the feature descriptor appended to the patch descriptor
        const cv::Mat_<float> fv = _fxs[j]->extract( _prect);
        if ( fv.empty())    // Unable to extract this feature vector type so can't use this patch
        {
            std::cerr << "[ERROR] PatchDescriptorExtractor::extractPatch: Feature not extracted!" << std::endl;
            assert( false);
        }   // end if
        pd->addRowFeatureVectors(fv);    // Rows are individual feature vectors (of equal length)
    }   // end for

    return pd;
}   // end extractPatch



// public
void PatchDescriptorExtractor::supplyDebugImage( const cv::Mat_<cv::Vec3b>& img)
{
    _timg = img.clone();
    if ( _timg.size() != _mask.size())
        throw RFeatures::ImageSizeException( "[ERROR] PatchDescriptorExtractor::supplyDebugImage: Supplied image dimensions do not match _mask dimensions!");
    cv::rectangle( _timg, _bbox, cv::Scalar(255,0,0), 2);
}   // end supplyDebugImage
