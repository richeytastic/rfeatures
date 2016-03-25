#include "Superpixels.h"
using RFeatures::Superpixels;
#include <cassert>
#include <cstdlib>
#include "FeatureUtils.h"



void convertTripleChannel( const cv::Mat &img, uint *buff)
{
    int k = 0;  // Index into buff
    for ( int i = 0; i < img.rows; ++i)
    {
        const cv::Vec3b* imgRow = img.ptr<cv::Vec3b>(i);
        for ( int j = 0; j < img.cols; ++j)
        {
            const cv::Vec3b &v = imgRow[j];
            const uint r = v[2];
            const uint g = v[1];
            const uint b = v[0];
            buff[k++] = 0xff0000 & (r << 16) | 0xff00 & (g << 8) | 0xff & b;
        }   // end for
    }   // end for
}   // end convertTripleChannel



void convertSingleChannel( const cv::Mat &img, uint *buff)
{
    int k = 0;  // Index into buff
    for ( int i = 0; i < img.rows; ++i)
    {
        const byte* imgRow = img.ptr<byte>(i);
        for ( int j = 0; j < img.cols; ++j)
        {
            const uint v = imgRow[j];   // Grey value spread over rgb channels
            buff[k++] = 0xff0000 & (v << 16) | 0xff00 & (v << 8) | 0xff & v;
        }   // end for
    }   // end for
}   // end convertSingleChannel



Superpixels::Superpixels( const cv::Mat &img,
                          int spParam, Superpixels::SPType spFlg,
                          int compactness)
    : imgSz_(img.size()), spFlag_(spFlg), spParam_(spParam), compactness_(compactness),
    segment_(NULL), imgBuff_(NULL), klabs_(NULL)
{
    assert( img.type() == CV_8UC3 || img.type() == CV_8UC1);
    imgBuff_ = new uint[img.total()];
    if ( img.channels() == 3)
        convertTripleChannel( img, imgBuff_);
    else
        convertSingleChannel( img, imgBuff_);
}   // end ctor



Superpixels::~Superpixels()
{
    if ( imgBuff_ != NULL)
        delete[] imgBuff_;
    if ( klabs_ != NULL)
        delete[] klabs_;
    if ( segment_ != NULL)
        delete segment_;
}   // end dtor



const int* const Superpixels::extract( int &numLabs)
{
    extract();
    numLabs = numLabs_;
    return klabs_;
}   // end extract



cv::Mat_<cv::Vec3b> Superpixels::createLabelImage( rlib::Random& rnd)
{
    extract();
    boost::unordered_map<int,cv::Vec3b> colourMap;

    cv::Mat labImg( imgSz_, CV_8UC3);
    const int sz = imgSz_.width * imgSz_.height;
    const int cols = imgSz_.width;
    for ( int i = 0; i < sz; ++i)
    {
        const int label = klabs_[i];
        if ( colourMap.count(label) != 1)
        {
            // Random values are at least 25,25,25 across channels (so not too dark)
            cv::Vec3b colour( (int)(230. * rnd.getRandom() + 25.5),
                              (int)(230. * rnd.getRandom() + 25.5),
                              (int)(230. * rnd.getRandom() + 25.5));
            colourMap[label] = colour;
        }   // end if

        labImg.at<cv::Vec3b>( i/cols, i%cols) = colourMap[label];
    }   // end for

    return labImg;
}   // end createLabelImage



cv::Mat_<byte> Superpixels::drawOutlines()
{
    extract();
    // Draw boundaries around the segments in the alpha channel of imgBuff_
    segment_->DrawContoursAroundSegments( imgBuff_, klabs_, imgSz_.width, imgSz_.height, 0xff000000);

    const int sz = imgSz_.width * imgSz_.height;
    cv::Mat img( 1, sz, CV_8UC1);
    byte *imgPtr = img.ptr<byte>(0);
    for ( int i = 0; i < sz; ++i)
    {
        const uint v = imgBuff_[i];
        imgPtr[i] = (byte)((v & 0xff000000) >> 24);
    }   // end for

    return img.reshape(0, imgSz_.height);
}   // end drawOutlines



// private
void Superpixels::extract()
{
    if ( segment_ != NULL)
        return;

    segment_ = new RFeatures::SLIC;
    if ( spFlag_ == Superpixels::SP_COUNT)
    {
        segment_->DoSuperpixelSegmentation_ForGivenNumberOfSuperpixels(
                imgBuff_, imgSz_.width, imgSz_.height, klabs_, numLabs_, spParam_, compactness_);
    }   // end if
    else
    {
        segment_->DoSuperpixelSegmentation_ForGivenSuperpixelSize(
                imgBuff_, imgSz_.width, imgSz_.height, klabs_, numLabs_, spParam_, compactness_);
    }   // end else
}   // end extract
