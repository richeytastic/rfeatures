#include "DepthDiff.h"
using RFeatures::DepthDiff;
using RFeatures::PatchPointType;
#include <algorithm>
#include <cmath>


int RFeatures::getPointPatchLength( const PatchPointType ddt)
{
    int len = 0;
    switch ( ddt)
    {
        case FOUR_PT:
            len = 6; break;
        case FIVE_PT:
            len = 10; break;
        case NINE_PT:
            len = 36; break;
        case THIRTEEN_PT:
            len = 78; break;
        default:
            len = 0;
    };  // end switch
    return len;
}   // end getPointPatchLength



DepthDiff::DepthDiff( const cv::Mat_<float>& dimg, PatchPointType ppt, float sensitivity)
    : RFeatures::FeatureOperator( dimg.size()), _img(dimg), _ppt(ppt), _sensitivity(sensitivity)
{}


// Maps depth differences in domain [-1,1] to [0,1].
float calcDepthDiff( float d, float sensitivity)
{
    const float baseVal = (d >= 0 ? 1 : -1) * std::min<float>( fabs(d), sensitivity) / sensitivity; // [-1,1]
    return (baseVal + 1)/2; // [0,1]
}   // end calcDepthDiff


// Sets 6 values in fv
void extractFourPoint( const cv::Mat_<float>& img, const cv::Rect& rct, cv::Mat_<float>& fv, float s)
{
    const int wseg = int( float(rct.width) / 3);
    const int hseg = int( float(rct.height) / 3);

    const float d0 = img.at<float>( rct.y + hseg, rct.x + wseg);
    const float d1 = img.at<float>( rct.y + hseg, rct.x + 2*wseg);
    const float d2 = img.at<float>( rct.y + 2*hseg, rct.x + wseg);
    const float d3 = img.at<float>( rct.y + 2*hseg, rct.x + 2*wseg);

    // 4*3/2 differences
    fv.push_back( calcDepthDiff( d0 - d1, s));
    fv.push_back( calcDepthDiff( d0 - d2, s));
    fv.push_back( calcDepthDiff( d0 - d3, s));

    fv.push_back( calcDepthDiff( d1 - d2, s));
    fv.push_back( calcDepthDiff( d1 - d3, s));

    fv.push_back( calcDepthDiff( d2 - d3, s));
}   // end extractFourPoint


// Sets 10 values in fv
void extractFivePoint( const cv::Mat_<float>& img, const cv::Rect& rct, cv::Mat_<float>& fv, float s)
{
    const float d0 = img.at<float>( rct.y, rct.x);
    const float d1 = img.at<float>( rct.y, rct.x + rct.width);
    const float d2 = img.at<float>( rct.y + rct.height, rct.x);
    const float d3 = img.at<float>( rct.y + rct.height, rct.x + rct.width);
    const float d4 = img.at<float>( rct.y + rct.height/2, rct.x + rct.width/2);

    // 5*4/2 differences
    fv.push_back( calcDepthDiff( d0 - d1, s));
    fv.push_back( calcDepthDiff( d0 - d2, s));
    fv.push_back( calcDepthDiff( d0 - d3, s));
    fv.push_back( calcDepthDiff( d0 - d4, s));

    fv.push_back( calcDepthDiff( d1 - d2, s));
    fv.push_back( calcDepthDiff( d1 - d3, s));
    fv.push_back( calcDepthDiff( d1 - d4, s));

    fv.push_back( calcDepthDiff( d2 - d3, s));
    fv.push_back( calcDepthDiff( d2 - d4, s));

    fv.push_back( calcDepthDiff( d3 - d4, s));
}   // end extractFivePoint


// Sets 36 values in fv
void extractNinePoint( const cv::Mat_<float>& img, const cv::Rect& rct, cv::Mat_<float>& fv, float s)
{
    const int x0 = rct.x;
    const int x1 = rct.x + rct.width/2;
    const int x2 = rct.x + rct.width;
    const int y0 = rct.y;
    const int y1 = rct.y + rct.height/2;
    const int y2 = rct.y + rct.height;

    const float d0 = img.at<float>( y0, x0);
    const float d1 = img.at<float>( y0, x1);
    const float d2 = img.at<float>( y0, x2);

    const float d3 = img.at<float>( y1, x0);
    const float d4 = img.at<float>( y1, x1);
    const float d5 = img.at<float>( y1, x2);

    const float d6 = img.at<float>( y2, x0);
    const float d7 = img.at<float>( y2, x1);
    const float d8 = img.at<float>( y2, x2);

    // 9*8/2 differences
    fv.push_back( calcDepthDiff( d0 - d1, s));
    fv.push_back( calcDepthDiff( d0 - d2, s));
    fv.push_back( calcDepthDiff( d0 - d3, s));
    fv.push_back( calcDepthDiff( d0 - d4, s));
    fv.push_back( calcDepthDiff( d0 - d5, s));
    fv.push_back( calcDepthDiff( d0 - d6, s));
    fv.push_back( calcDepthDiff( d0 - d7, s));
    fv.push_back( calcDepthDiff( d0 - d8, s));

    fv.push_back( calcDepthDiff( d1 - d2, s));
    fv.push_back( calcDepthDiff( d1 - d3, s));
    fv.push_back( calcDepthDiff( d1 - d4, s));
    fv.push_back( calcDepthDiff( d1 - d5, s));
    fv.push_back( calcDepthDiff( d1 - d6, s));
    fv.push_back( calcDepthDiff( d1 - d7, s));
    fv.push_back( calcDepthDiff( d1 - d8, s));

    fv.push_back( calcDepthDiff( d2 - d3, s));
    fv.push_back( calcDepthDiff( d2 - d4, s));
    fv.push_back( calcDepthDiff( d2 - d5, s));
    fv.push_back( calcDepthDiff( d2 - d6, s));
    fv.push_back( calcDepthDiff( d2 - d7, s));
    fv.push_back( calcDepthDiff( d2 - d8, s));

    fv.push_back( calcDepthDiff( d3 - d4, s));
    fv.push_back( calcDepthDiff( d3 - d5, s));
    fv.push_back( calcDepthDiff( d3 - d6, s));
    fv.push_back( calcDepthDiff( d3 - d7, s));
    fv.push_back( calcDepthDiff( d3 - d8, s));

    fv.push_back( calcDepthDiff( d4 - d5, s));
    fv.push_back( calcDepthDiff( d4 - d6, s));
    fv.push_back( calcDepthDiff( d4 - d7, s));
    fv.push_back( calcDepthDiff( d4 - d8, s));

    fv.push_back( calcDepthDiff( d5 - d6, s));
    fv.push_back( calcDepthDiff( d5 - d7, s));
    fv.push_back( calcDepthDiff( d5 - d8, s));

    fv.push_back( calcDepthDiff( d6 - d7, s));
    fv.push_back( calcDepthDiff( d6 - d8, s));

    fv.push_back( calcDepthDiff( d7 - d8, s));
}   // end extractNinePoint


// Sets 78 values in fv
void extractThirteenPoint( const cv::Mat_<float>& img, const cv::Rect& rct, cv::Mat_<float>& fv, float s)
{
    const int x0 = rct.x;
    const int x1 = rct.x + rct.width/2;
    const int x2 = rct.x + rct.width;
    const int x3 = x0 + (x1-x0)/2;
    const int x4 = x1 + (x2-x1)/2;

    const int y0 = rct.y;
    const int y1 = rct.y + rct.height/2;
    const int y2 = rct.y + rct.height;
    const int y3 = y0 + (y1-y0)/2;
    const int y4 = y1 + (y2-y1)/2;

    const float d0 = img.at<float>( y0, x0);
    const float d1 = img.at<float>( y0, x1);
    const float d2 = img.at<float>( y0, x2);

    const float d3 = img.at<float>( y1, x0);
    const float d4 = img.at<float>( y1, x1);
    const float d5 = img.at<float>( y1, x2);

    const float d6 = img.at<float>( y2, x0);
    const float d7 = img.at<float>( y2, x1);
    const float d8 = img.at<float>( y2, x2);

    const float d9 = img.at<float>( y3, x3);
    const float d10 = img.at<float>( y3, x4);
    const float d11 = img.at<float>( y4, x3);
    const float d12 = img.at<float>( y4, x4);

    // 13*12/2 differences
    fv.push_back( calcDepthDiff( d0 - d1, s));
    fv.push_back( calcDepthDiff( d0 - d2, s));
    fv.push_back( calcDepthDiff( d0 - d3, s));
    fv.push_back( calcDepthDiff( d0 - d4, s));
    fv.push_back( calcDepthDiff( d0 - d5, s));
    fv.push_back( calcDepthDiff( d0 - d6, s));
    fv.push_back( calcDepthDiff( d0 - d7, s));
    fv.push_back( calcDepthDiff( d0 - d8, s));
    fv.push_back( calcDepthDiff( d0 - d9, s));
    fv.push_back( calcDepthDiff( d0 - d10, s));
    fv.push_back( calcDepthDiff( d0 - d11, s));
    fv.push_back( calcDepthDiff( d0 - d12, s));

    fv.push_back( calcDepthDiff( d1 - d2, s));
    fv.push_back( calcDepthDiff( d1 - d3, s));
    fv.push_back( calcDepthDiff( d1 - d4, s));
    fv.push_back( calcDepthDiff( d1 - d5, s));
    fv.push_back( calcDepthDiff( d1 - d6, s));
    fv.push_back( calcDepthDiff( d1 - d7, s));
    fv.push_back( calcDepthDiff( d1 - d8, s));
    fv.push_back( calcDepthDiff( d1 - d9, s));
    fv.push_back( calcDepthDiff( d1 - d10, s));
    fv.push_back( calcDepthDiff( d1 - d11, s));
    fv.push_back( calcDepthDiff( d1 - d12, s));

    fv.push_back( calcDepthDiff( d2 - d3, s));
    fv.push_back( calcDepthDiff( d2 - d4, s));
    fv.push_back( calcDepthDiff( d2 - d5, s));
    fv.push_back( calcDepthDiff( d2 - d6, s));
    fv.push_back( calcDepthDiff( d2 - d7, s));
    fv.push_back( calcDepthDiff( d2 - d8, s));
    fv.push_back( calcDepthDiff( d2 - d9, s));
    fv.push_back( calcDepthDiff( d2 - d10, s));
    fv.push_back( calcDepthDiff( d2 - d11, s));
    fv.push_back( calcDepthDiff( d2 - d12, s));

    fv.push_back( calcDepthDiff( d3 - d4, s));
    fv.push_back( calcDepthDiff( d3 - d5, s));
    fv.push_back( calcDepthDiff( d3 - d6, s));
    fv.push_back( calcDepthDiff( d3 - d7, s));
    fv.push_back( calcDepthDiff( d3 - d8, s));
    fv.push_back( calcDepthDiff( d3 - d9, s));
    fv.push_back( calcDepthDiff( d3 - d10, s));
    fv.push_back( calcDepthDiff( d3 - d11, s));
    fv.push_back( calcDepthDiff( d3 - d12, s));

    fv.push_back( calcDepthDiff( d4 - d5, s));
    fv.push_back( calcDepthDiff( d4 - d6, s));
    fv.push_back( calcDepthDiff( d4 - d7, s));
    fv.push_back( calcDepthDiff( d4 - d8, s));
    fv.push_back( calcDepthDiff( d4 - d9, s));
    fv.push_back( calcDepthDiff( d4 - d10, s));
    fv.push_back( calcDepthDiff( d4 - d11, s));
    fv.push_back( calcDepthDiff( d4 - d12, s));

    fv.push_back( calcDepthDiff( d5 - d6, s));
    fv.push_back( calcDepthDiff( d5 - d7, s));
    fv.push_back( calcDepthDiff( d5 - d8, s));
    fv.push_back( calcDepthDiff( d5 - d9, s));
    fv.push_back( calcDepthDiff( d5 - d10, s));
    fv.push_back( calcDepthDiff( d5 - d11, s));
    fv.push_back( calcDepthDiff( d5 - d12, s));

    fv.push_back( calcDepthDiff( d6 - d7, s));
    fv.push_back( calcDepthDiff( d6 - d8, s));
    fv.push_back( calcDepthDiff( d6 - d9, s));
    fv.push_back( calcDepthDiff( d6 - d10, s));
    fv.push_back( calcDepthDiff( d6 - d11, s));
    fv.push_back( calcDepthDiff( d6 - d12, s));

    fv.push_back( calcDepthDiff( d7 - d8, s));
    fv.push_back( calcDepthDiff( d7 - d9, s));
    fv.push_back( calcDepthDiff( d7 - d10, s));
    fv.push_back( calcDepthDiff( d7 - d11, s));
    fv.push_back( calcDepthDiff( d7 - d12, s));

    fv.push_back( calcDepthDiff( d8 - d9, s));
    fv.push_back( calcDepthDiff( d8 - d10, s));
    fv.push_back( calcDepthDiff( d8 - d11, s));
    fv.push_back( calcDepthDiff( d8 - d12, s));

    fv.push_back( calcDepthDiff( d9 - d10, s));
    fv.push_back( calcDepthDiff( d9 - d11, s));
    fv.push_back( calcDepthDiff( d9 - d12, s));

    fv.push_back( calcDepthDiff( d10 - d11, s));
    fv.push_back( calcDepthDiff( d10 - d12, s));

    fv.push_back( calcDepthDiff( d11 - d12, s));
}   // end extractThirteenPoint



cv::Mat_<float> DepthDiff::extract( const cv::Rect& rct) const
{
    cv::Mat_<float> fv;
    switch ( _ppt)
    {
        case FOUR_PT:
            extractFourPoint( _img, rct, fv, _sensitivity);
            break;
        case FIVE_PT:
            extractFivePoint( _img, rct, fv, _sensitivity);
            break;
        case NINE_PT:
            extractNinePoint( _img, rct, fv, _sensitivity);
            break;
        case THIRTEEN_PT:
            extractThirteenPoint( _img, rct, fv, _sensitivity);
            break;
    }   // end switch

    // Transpose to row vector
    const cv::Mat_<float> fv2 = fv.t();
    return fv2;
}   // end extract

