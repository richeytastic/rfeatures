#include "LocalBinaryPattern.h"
using RFeatures::LocalBinaryPattern;
#include <cassert>


LocalBinaryPattern::LocalBinaryPattern( const cv::Mat& img)
    : RFeatures::FeatureOperator( img.size()), _imgRct( 0, 0, img.cols, img.rows)
{
    assert( img.channels() == 1);
    assert( img.isContinuous());
    cv::Mat iimg;
    img.convertTo( iimg, CV_32F);
    cv::integral( iimg, _intImg, CV_64F);
}   // end ctor


LocalBinaryPattern::~LocalBinaryPattern()
{ }   // end dtor


bool testThreshold( const cv::Mat_<double>& ii, const cv::Rect& rct, double thresh)
{
    return ( RFeatures::getIntegralImageSum<double>( ii, rct) / rct.area()) >= thresh;
}   // end testThreshold



// public
byte LocalBinaryPattern::calcBitPattern( const cv::Rect& rct) const
{
    if ( (_imgRct & rct) != rct)
        return 0;

    int wdim = rct.width/3;
    int hdim = rct.height/3;
    int mwdim = rct.width - 2*wdim;
    int mhdim = rct.height - 2*hdim;

    // |0|1|2|
    // |3|4|5|
    // |6|7|8|
    std::vector<cv::Rect> rcts(9);
    rcts[0] = cv::Rect( rct.x,                rct.y,                wdim,  hdim);
    rcts[1] = cv::Rect( rct.x + wdim,         rct.y,                mwdim, hdim);
    rcts[2] = cv::Rect( rct.x + wdim + mwdim, rct.y,                wdim,  hdim);

    rcts[3] = cv::Rect( rct.x,                rct.y + hdim,         wdim,  mhdim);
    rcts[4] = cv::Rect( rcts[1].x,            rcts[3].y,            mwdim, mhdim);
    rcts[5] = cv::Rect( rcts[2].x,            rcts[3].y,            wdim,  mhdim);

    rcts[6] = cv::Rect( rct.x,                rct.y + hdim + mhdim, wdim,  hdim);
    rcts[7] = cv::Rect( rcts[1].x,            rcts[6].y,            mwdim, hdim);
    rcts[8] = cv::Rect( rcts[2].x,            rcts[6].y,            wdim,  hdim);


    double thresh = RFeatures::getIntegralImageSum<double>( _intImg, rcts[4]) / rcts[4].area(); // Get the middle square threshold

    byte v = 0;
    if ( testThreshold( _intImg, rcts[0], thresh))  // Top left
        v |= 0x01;

    v <<= 1;
    if ( testThreshold( _intImg, rcts[1], thresh))  // Top middle
        v |= 0x01;

    v <<= 1;
    if ( testThreshold( _intImg, rcts[2], thresh))  // Top right
        v |= 0x01;

    v <<= 1;
    if ( testThreshold( _intImg, rcts[5], thresh))  // Middle right
        v |= 0x01;

    v <<= 1;
    if ( testThreshold( _intImg, rcts[8], thresh))  // Bottom right
        v |= 0x01;

    v <<= 1;
    if ( testThreshold( _intImg, rcts[7], thresh))  // Bottom middle
        v |= 0x01;

    v <<= 1;
    if ( testThreshold( _intImg, rcts[6], thresh))  // Bottom left
        v |= 0x01;

    v <<= 1;
    if ( testThreshold( _intImg, rcts[3], thresh))  // Middle left
        v |= 0x01;

    return v;
}   // end calcBitPattern


// public
cv::Vec3b LocalBinaryPattern::extractBinaryPattern( const cv::Rect& rct) const
{
    cv::Rect r0(rct);

    int incWDim = rct.width % 3;
    if ( incWDim == 1)
        r0.width--;
    else if ( incWDim == 2)
        r0.width++;

    int incHDim = rct.height % 3;
    if ( incHDim == 1)
        r0.height--;
    else if ( incHDim == 2)
        r0.height++;

    // Make the two other size up rectangles using a dimensiom increase that
    // allows for division by three in the dimensions of the larger rectangles.
    const int wdim = r0.width % 2 ? r0.width/2 : (r0.width-1)/2 + 2;
    const int hdim = r0.height % 2 ? r0.height/2 : (r0.height-1)/2 + 2;

    const cv::Rect r1( r0.x-wdim/2-1, r0.y-hdim/2-1,   r0.width+wdim,   r0.height+hdim); // 1.5X
    const cv::Rect r2( r0.x-wdim,     r0.y-hdim,     2*r0.width,      2*r0.height); // 2.0X

    return cv::Vec3b( calcBitPattern( r0), calcBitPattern( r1), calcBitPattern( r2));
}   // end extractBinaryPattern


// public
cv::Mat_<float> LocalBinaryPattern::extract( const cv::Rect& rct) const
{
    const cv::Vec3b fv = extractBinaryPattern(rct); // 3-bytes
    cv::Mat_<float> fvm(1,1);
    *fvm.ptr<float>() = *(float*)&fv[0];    // Cast to a single float (4-bytes) for return
    return fvm;
}   // end extract



cv::Mat LocalBinaryPattern::map( const cv::Mat& img, const std::vector<size_t>& dims)
{
    assert( img.channels() == 1);
    assert( img.isContinuous());
    LocalBinaryPattern fx( img);
    const int rows = img.rows;
    const int cols = img.cols;

    const int nc = (int)dims.size();
    cv::Mat outMap( rows, cols, CV_8UC(nc));

    // Create the rectangles
    std::vector<cv::Rect> rcts(nc);
    std::vector<size_t> hdims(nc);
    for ( int c = 0; c < nc; ++c)
    {
        rcts[c].width = (int)dims[c];
        rcts[c].height = (int)dims[c];
        hdims[c] = dims[c]/2;
    }   // end for

    for ( int i = 0; i < rows; ++i)
    {
        byte *outRow = outMap.ptr(i);

        for ( int c = 0; c < nc; ++c)
            rcts[c].y = i - (int)hdims[c];

        for ( int j = 0; j < cols; ++j)
        {
            const int colIdx = j*nc;
            for ( int c = 0; c < nc; ++c)
            {
                cv::Rect& rct = rcts[c];
                rct.x = j - (int)hdims[c];
                outRow[colIdx + c] = fx.calcBitPattern( rct);
            }   // end for
        }   // end for - cols
    }   // end for - rows

    return outMap;
}   // end createLocalBinaryPattern
