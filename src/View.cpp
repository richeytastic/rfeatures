#include "View.h"
using RFeatures::View;
#include <cmath>
#include <cassert>
#include <algorithm>
#include <string>
using std::string;
using std::getline;
#include <sstream>
using std::istringstream;
#include <iostream>
using std::cerr;
using std::endl;


View::Ptr View::create( const cv::Size& sz)
{
    return View::Ptr( new View( sz));
}   // end create


View::Ptr View::create( int width, int height)
{
    return View::Ptr( new View( width, height));
}   // end create


View::Ptr View::create( cv::Mat_<cv::Vec3b> img, cv::Mat_<float> rng)
{
    return View::Ptr( new View( img, rng));
}   // end create


View::View( const cv::Size& sz)
{
    img2d.create(sz);
    points.create(sz);
    rngImg.create(sz);
}   // end ctor


View::View( int width, int height)
{
    img2d.create(height, width);
    points.create(height, width);
    rngImg.create(height, width);
}   // end ctor


View::View( cv::Mat_<cv::Vec3b> img, cv::Mat_<float> rng) : img2d(img), rngImg(rng)
{
    points.create(img.size());
}   // end ctor


cv::Size View::size() const
{
    return img2d.size();
}   // end size



cv::Mat_<byte> RFeatures::makeDisplayableRangeMap( const cv::Mat_<float>& rngImg, float minRng, float maxRng)
{
    // rngImg zero values should not be included
    minRng = std::max<float>(0, minRng);

    if ( maxRng < minRng)
    {
        double mx, notused;
        cv::minMaxLoc( rngImg, &notused, &mx);
        maxRng = mx;
    }   // end else

    const cv::Mat rngMask = (rngImg >= minRng) & (rngImg <= maxRng) & (rngImg != 0);    // Mask valid range values

    cv::Mat_<float> trng = cv::Mat_<float>::ones( rngImg.size()) * maxRng;
    rngImg.copyTo( trng, rngMask);

    cv::Mat_<byte> outImg;
    trng.convertTo( outImg, CV_8U, -255./maxRng, 255);  // Invert
    return outImg;
}   // end makeDisplayableRangeMap



void readImageData( istream& is, cv::Mat_<cv::Vec3b>& cimg, cv::Mat_<cv::Vec3f>& points, cv::Mat_<float>& dimg)
{
    string ln;
    getline( is, ln);
    istringstream iss(ln);

    cv::Size imgSz;
    iss >> imgSz.height >> imgSz.width;
    cimg = cv::Mat_<cv::Vec3b>( imgSz);
    points = cv::Mat_<cv::Vec3f>( imgSz);
    dimg = cv::Mat_<float>( imgSz);

    const int sz = imgSz.width * imgSz.height;
    const int pxlChunk = 3*sizeof(float) + 3*sizeof(byte);
    const int totalBytes = sz * pxlChunk;
    char* buff = (char*)malloc( totalBytes);

    int readBytes = 0;
    while ( readBytes < totalBytes)
    {
        is.read( &buff[readBytes], totalBytes-readBytes);
        const int numBytesRead = is.gcount();
        if ( numBytesRead <= 0)
            break;
        readBytes += numBytesRead;
    }   // end while

    assert( readBytes == totalBytes);

    for ( int i = 0; i < sz; ++i)
    {
        int j = i * pxlChunk;   // Offset into read in buffer

        // Read in points (with respect to origin)
        cv::Vec3f p( *(float*)&buff[j], // X
                     *(float*)&buff[j+sizeof(float)], // Y
                     *(float*)&buff[j+2*sizeof(float)]);    // Z (depth)

        j += 3*sizeof(float);   // Skip to colour bytes
        cv::Vec3b c( (byte)buff[j], (byte)buff[j+1], (byte)buff[j+2]);

        const int row = i / imgSz.width;  // Integer division
        const int col = i % imgSz.width;

        cimg.at<cv::Vec3b>(row,col) = c;
        points.at<cv::Vec3f>(row,col) = p;
        dimg.at<float>(row,col) = p[2]; // Depth is just the Z value
    }   // end for

    free(buff);

    getline( is, ln);  // Read end of line
}   // end readImageData



void writeImageData( ostream& os, const cv::Mat_<cv::Vec3b>& cimg, const cv::Mat_<cv::Vec3f>& points)
{
    const cv::Size imgSz = cimg.size();
    assert( imgSz == points.size());

    os << imgSz.height << " " << imgSz.width << endl;
    for ( int i = 0; i < imgSz.height; ++i)
    {
        const cv::Vec3f* pptr = points.ptr<cv::Vec3f>(i);
        const cv::Vec3b* cptr = cimg.ptr<cv::Vec3b>(i);
        for ( int j = 0; j < imgSz.width; ++j)
        {
            const cv::Vec3f& p = pptr[j];
            const cv::Vec3b& c = cptr[j];

            // Write the x,y,z
            os.write( (char*)&p[0], sizeof(float));
            os.write( (char*)&p[1], sizeof(float));
            os.write( (char*)&p[2], sizeof(float));

            // Write the colour
            os.write( (char*)&c[0], sizeof(byte));
            os.write( (char*)&c[1], sizeof(byte));
            os.write( (char*)&c[2], sizeof(byte));
        }   // end for - columns
    }   // end for - rows

    os << endl;
}   // end writeImageData



istream& RFeatures::operator>>( istream& is, View::Ptr& v)
{
    cv::Vec3d posVec, focalVec, upVec;
    is >> posVec[0] >> posVec[1] >> posVec[2];
    is >> focalVec[0] >> focalVec[1] >> focalVec[2];
    is >> upVec[0] >> upVec[1] >> upVec[2];

    // **** Deprecated params ****
    int nbins;
    bool dd, ss, gc;    // Direction dependence, spatial smoothing and gamma correction flags
    is >> nbins >> dd >> ss >> gc;  // For 2D image
    is >> nbins >> dd >> ss;    // For range image
    // ***************************

    string eol; // Discard end of line
    getline(is,eol);

    cv::Mat_<cv::Vec3b> img2d;
    cv::Mat_<cv::Vec3f> points;
    cv::Mat_<float> rngImg;
    readImageData( is, img2d, points, rngImg);

    v = View::create( 0,0);
    v->posVec = posVec;
    v->focalVec = focalVec;
    v->upVec = upVec;
    v->img2d = img2d;
    v->points = points;
    v->rngImg = rngImg;

    return is;
}   // end operator>>



ostream& RFeatures::operator<<( ostream& os, const View::Ptr& v)
{
    const cv::Vec3d &pv = v->posVec; // View position
    os << pv[0] << " " << pv[1] << " " << pv[2] << endl;
    const cv::Vec3d &fv = v->focalVec; // View focal vector
    os << fv[0] << " " << fv[1] << " " << fv[2] << endl;
    const cv::Vec3d &uv = v->upVec; // View up vector
    os << uv[0] << " " << uv[1] << " " << uv[2] << endl;

    const int n(0);
    const bool b(false);
    os << n << " " << b << " " << b << " " << b << endl;    // Deprecated params
    os << n << " " << b << " " << b << endl;    // Deprecated params
    /*
    // Image gradients parameters
    os << (v->imgGrads->channels() - 1) << " " << v->imgDirDep
        << " " << v->imgSpatialSmooth << " " << v->sqrtGammaCorrect << endl;
    // Range gradients parameters
    os << (v->rngGrads->channels() - 1) << " " << v->rngDirDep << " " << v->rngSpatialSmooth << endl;
    */

    writeImageData( os, v->img2d, v->points);
    return os;
}   // end operator<<



void RFeatures::createChangeMaps( const cv::Mat& img, cv::Mat& hchng, cv::Mat& vchng, bool useAbsoluteValue, cv::Mat_<byte> msk)
{
    assert( img.channels() == 1);
    const int depth = img.depth();
    switch ( depth)
    {
        case CV_8U:
            RFeatures::__createChangeMaps<byte>( img, hchng, vchng, useAbsoluteValue, msk);
            break;
        case CV_8S:
            RFeatures::__createChangeMaps<char>( img, hchng, vchng, useAbsoluteValue, msk);
            break;
        case CV_16S:
            RFeatures::__createChangeMaps<int16_t>( img, hchng, vchng, useAbsoluteValue, msk);
            break;
        case CV_16U:
            RFeatures::__createChangeMaps<uint16_t>( img, hchng, vchng, useAbsoluteValue, msk);
            break;
        case CV_32S:
            RFeatures::__createChangeMaps<int32_t>( img, hchng, vchng, useAbsoluteValue, msk);
            break;
        case CV_32F:
            RFeatures::__createChangeMaps<float>( img, hchng, vchng, useAbsoluteValue, msk);
            break;
        case CV_64F:
            RFeatures::__createChangeMaps<double>( img, hchng, vchng, useAbsoluteValue, msk);
            break;
        default:
            throw RFeatures::ImageTypeException("[ERROR] RangeGradientExtractor::ctor: Invalid image type!");
    }   // end switch
}   // end createChangeMaps
