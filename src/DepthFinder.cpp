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

#include "DepthFinder.h"
using RFeatures::DepthFinder;
#include <cmath>
typedef unsigned char byte;


const double DepthFinder::MAX_DEPTH = FLT_MAX;  // metres for points with do depth
double DepthFinder::camHeight = 2.7;



DepthFinder::Ptr DepthFinder::create( const cv::Mat_<double> &dii, const cv::Mat_<int> &cii)
{
    DepthFinder *df = new DepthFinder( dii, cii);
    return DepthFinder::Ptr( df);
}   // end create



DepthFinder::DepthFinder( const cv::Mat_<double> &dii, const cv::Mat_<int> &cii)
    : imgRct_( 0, 0, dii.cols-1, dii.rows-1), depthIntImg(dii), depthCntImg(cii)
{ }   // end ctor



DepthFinder::DepthFinder( const cv::Mat_<cv::Vec3f>& pc, const cv::Vec3d &focVec)
{
    imgRct_ = cv::Rect( 0, 0, pc.cols, pc.rows);
    // Integral image with single channel for depth
    const cv::Size imgSz( imgRct_.width, imgRct_.height);

    cv::Mat_<double> dmap(imgSz);
    cv::Mat_<byte> dcnts(imgSz);

    for ( int i = 0; i < imgSz.height; ++i)
    {
        double* drow = dmap.ptr<double>(i);
        byte* irow = dcnts.ptr<byte>(i);

        for ( int j = 0; j < imgSz.width; ++j)
        {
            const cv::Vec3f& fp = pc.at<cv::Vec3f>(i,j);
            cv::Vec3d p( fp[0], fp[1], fp[2]);
            double depth = focVec.dot( p);
            if ( depth < 0)
                depth = 0;
            if ( depth > MAX_DEPTH)
                depth = MAX_DEPTH;

            drow[j] = depth;
            irow[j] = depth > 0 ? 1 : 0; // Only points with valid depth are set
        }   // end for
    }   // end for

    cv::integral( dmap, depthIntImg, CV_64F);
    cv::integral( dcnts, depthCntImg, CV_32S);
}   // end ctor



int DepthFinder::getValidRangeCount( const cv::Rect &rct) const
{
    assert( (rct & imgRct_) == rct);
    return RFeatures::getIntegralImageSum<int>( depthCntImg, rct);  // Number of points with valid depth
}   // end getValidRangeCount



double DepthFinder::getAvgDepth( const cv::Rect &rct) const
{
    assert( (rct & imgRct_) == rct);
    if ( rct.height == 0 || rct.width == 0)
        return MAX_DEPTH;
    // Get the average depth over those parts of rct having valid depth info
    const int cnt = getValidRangeCount( rct);
    if ( cnt == 0)  // No points at this location with valid depth, so return max depth
        return MAX_DEPTH;
    return RFeatures::getIntegralImageSum<double>( depthIntImg, rct) / cnt;   // Average depth
}   // end getAvgDepth



double DepthFinder::getDepth( int row, int col) const
{
    const cv::Rect rct(col,row,1,1);    // Single pixel
    if ( getValidRangeCount( rct) == 0)  // Number of points with valid depth
        return 0;
    return RFeatures::getIntegralImageSum<double>( depthIntImg, rct);   // depth
}   // end getDepth



double DepthFinder::getMeanDepthAlongBase( const cv::Rect &boundBox, float maxDep) const
{
    const cv::Rect bb = boundBox & imgRct_;

    double depthSum = 0;
    int vcount = 0;

    const int row = bb.y + bb.height - 1;   // Row along base of bb
    const int colMax = bb.x + bb.width;
    for ( int i = bb.x; i < colMax; ++i)
    {
        const float d = (float)getDepth( row, i);
        if ( d > 0 && d < maxDep)
        {
            depthSum += d;
            vcount++;
        }   // end if
    }   // end for

    return vcount == 0 ? 0 : depthSum / vcount;
}   // end getMeanDepthAlongBase



double DepthFinder::calcSize( const cv::Rect &bb, cv::Size2f &actSz) const
{
    const double depth = getAvgDepth( bb);
    const double widthProp = (double)bb.width / imgRct_.width;
    const double heightProp = (double)bb.height / imgRct_.height;
    actSz.width = float(widthProp * 2*depth);
    actSz.height = float(heightProp * 2*depth);
    return depth;
}   // end calcSize



cv::Rect DepthFinder::getModelRect( const cv::Size2f &modelSz, int x, int y, int testSz) const
{
    cv::Rect modelRect;
    calcModelRect( modelSz, x, y, modelRect, testSz);
    return modelRect;
}   // end getModelRect



double DepthFinder::calcModelRect( const cv::Size2f &modelSz, int x, int y, cv::Rect &modelRect, int testRad) const
{
    // First assess initial depth over a fixed square pixel region with radius testRad
    if ( testRad < 1)
        testRad = 1;

    double depthVal = MAX_DEPTH;

    const int testRad2 = 2*testRad;
    // Intersection with image rectangle
    cv::Rect rct = cv::Rect( x - testRad, y - testRad, testRad2, testRad2) & imgRct_;
    // If none of this rectangle intersects with the image, return the zero sized rectangle
    if ( rct.width == 0 || rct.height == 0)
    {
        modelRect.width = 0;
        modelRect.height = 0;
    }   // end if
    else
    {
        const int cnt = getValidRangeCount( rct);  // Number of points with valid depth
        if ( cnt > 0)  // No points at this location with valid depth, so return max depth
            depthVal = getAvgDepth( rct) / cnt;   // Average depth

        calcScaleModel( modelSz, y, x, depthVal, 90, modelRect);
    }   // end else

    return depthVal;
}   // end calcModelRect



void DepthFinder::calcScaleModel( const cv::Size2f &modelSz, int row, int col,
                            double depth, double fov, cv::Rect &modelRect) const
{
    const double opp = (double)imgRct_.height/2;
    double f = opp;
    if ( fabs(fov - 90) > 0.00001) // Calculate the focal length if not 90 degrees
    {
        double hyp = opp/sin(CV_PI/2 * fov/180);
        f = sqrt(hyp*hyp - opp*opp);
    }   // end if

    // Calculate the apparent height of the model in pixel rows
    const double mRows = f*modelSz.height/depth;  // By similar triangles

    // Calculate the apparent width of the model in pixel columns
    const double mCols = modelSz.width * mRows/modelSz.height;  // By proportion

    // Calculate new bounds of the model in the image centred at this pixel
    modelRect.x = cvRound((double)col - mCols/2); // x pos - may be off image!
    modelRect.y = cvRound((double)row - mRows/2); // y pos - may be off image!
    modelRect.width = cvRound( mCols);
    modelRect.height = cvRound( mRows);
}   // end calcScaleModel



// static
void DepthFinder::setCameraHeight( double ch)
{
    camHeight = ch;
}   // end setCameraHeight
