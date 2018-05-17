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

#include <CircleDiff.h>
#include <cmath>
#include <cstdlib>
#include <cassert>
using RFeatures::CircleDiff;

#ifndef M_PI
#define M_PI 3.14159265359
#endif


struct CircleDiff::ImageCircleDiffs
{
    const int npoints;
    vector<double> cdiffs[3];

    double maxSecDiff;
    int maxIdx;

    double sums[3];
    double cvs[3];  // Previous circle values (around the circumference)

    ImageCircleDiffs( int nps) : npoints(nps)
    {
        maxSecDiff = -FLT_MAX;
        maxIdx = 0;

        sums[0] = 0;
        sums[1] = 0;
        sums[2] = 0;
        cvs[0] = 0;
        cvs[1] = 0;
        cvs[2] = 0;

        cdiffs[0].resize(npoints);
        cdiffs[1].resize(npoints);
        cdiffs[2].resize(npoints);
    }   // end init

    void updateCircle( int cIdx, int pIdx, double v)
    {
        //assert( cIdx >= 0 && cIdx < 3);
        //assert( pIdx >= 0 && pIdx < npoints);
        const double d = cdiffs[cIdx][pIdx] = v - cvs[cIdx];    // Store
        cvs[cIdx] = v;    // For next iteration

        // Record the maximum index over all circles
        if ( d > maxSecDiff)
        {
            maxSecDiff = d;
            maxIdx = pIdx;
        }   // end if

        sums[cIdx] += fabs(d); // For normalisation
    }   // end updateCircle
};  // end struct



void createCirclePoints( double r, double piOffset, int nps, vector<cv::Point2d>& pts)
{
    const double piChunk = 2*M_PI/nps;
    double piVal = piOffset;
    for ( int i = 0; i < nps; ++i)
    {
        cv::Point2d p( r*cos(piVal), r*sin(piVal));
        pts.push_back(p);
        piVal += piChunk;
    }   // end for
}   // end createCirclePoints



CircleDiff::CircleDiff( const cv::Mat_<float>& img, int nps)
    : RFeatures::FeatureOperator(img.size()), _img(img), _numPoints(nps), _icds( new ImageCircleDiffs(nps))
{
    createCirclePoints( 1, 0, nps, _points[0]);  // Outer circle
    // Offset inner circles by small angle increments
    createCirclePoints( 2./3, M_PI/12, nps, _points[1]); // Middle circle
    createCirclePoints( 1./3, M_PI/6, nps, _points[2]);  // Inner circle
}   // end ctor



CircleDiff::~CircleDiff()
{
    delete _icds;
}   // end dtor


/*
void CircleDiff::addImage( const cv::Mat_<float> img)
{
    assert( img.isContinuous());
    assert( img.type() == CV_32FC1);
    if ( !_imgs.empty())
    {
        assert( img.type() == _imgs[0].type());
        assert( img.size() == _imgs[0].size());
    }   // end if
    _imgs.push_back(img);
}   // end addImage
*/


/*
cv::Mat_<byte> CircleDiff::showPattern( const cv::Size& imgSz) const
{
    cv::Mat_<float> fv(1, 3*_numPoints);
    float* fvPtr = fv.ptr<float>();
    for ( int i = 0; i < _numPoints; ++i)
    {
        fvPtr[i] = 1;
        fvPtr[i+_numPoints] = 1;
        fvPtr[i+2*_numPoints] = 1;
    }   // end for
    return visFeature( fv, imgSz);
}   // end showPattern
*/


cv::Mat_<float> CircleDiff::extract( const cv::Rect& rct) const
{
    const int ncols = _img.cols;
    const int nrows = _img.rows;

    const double rowRad = rct.height/2-1;
    const double colRad = rct.width/2-1;
    const int y = rct.y + int(rowRad);
    const int x = rct.x + int(colRad);

    // Circle radius levels
    const vector<cv::Point2d>& points0 = _points[0];
    const vector<cv::Point2d>& points1 = _points[1];
    const vector<cv::Point2d>& points2 = _points[2];
    const cv::Point2d& p01 = points0[_numPoints-1];
    const cv::Point2d& p11 = points1[_numPoints-1];
    const cv::Point2d& p21 = points2[_numPoints-1];

    _icds->cvs[0] = _img.at<float>((int)(y + p01.y*rowRad), (int)(x + p01.x*colRad)); // Circle 0
    _icds->cvs[1] = _img.at<float>((int)(y + p11.y*rowRad), (int)(x + p11.x*colRad)); // Circle 1
    _icds->cvs[2] = _img.at<float>((int)(y + p21.y*rowRad), (int)(x + p21.x*colRad)); // Circle 2

    const cv::Rect imgRct( 0, 0, ncols, nrows); // For DEBUG
    cv::Point p;
    for ( int i = 0; i < _numPoints; ++i)
    {
        const cv::Point2d& p0 = points0[i]; // Circle level 0
        p = cv::Point( (int)(y + p0.y*rowRad), (int)(x + p0.x*colRad));
        assert( imgRct.contains(p));
        _icds->updateCircle( 0, i, _img.at<float>( p));

        const cv::Point2d& p1 = points1[i]; // Circle level 1
        p = cv::Point( (int)(y + p1.y*rowRad), (int)(x + p1.x*colRad));
        assert( imgRct.contains(p));
        _icds->updateCircle( 1, i, _img.at<float>( p));

        const cv::Point2d& p2 = points2[i]; // Circle level 2
        p = cv::Point( (int)(y + p2.y*rowRad), (int)(x + p2.x*colRad));
        assert( imgRct.contains(p));
        _icds->updateCircle( 2, i, _img.at<float>( p));
    }   // end for

    const int maxIdx = _icds->maxIdx;
    const vector<double>* cdiffs = _icds->cdiffs;
    const double* sums = _icds->sums;

    // Set the output vector, normalising the differences (but maintaining the sign)
    cv::Mat_<float> valsMat( 1, 3*_numPoints);
    float *vals = valsMat.ptr<float>();
    for ( int j = 0; j < _numPoints; ++j)
    {
        const int idx = (j+maxIdx) % _numPoints;
        const double a = cdiffs[0][idx] / (2*sums[0]) + 0.5;  // Between 0 and 1
        const double b = cdiffs[1][idx] / (2*sums[1]) + 0.5;
        const double c = cdiffs[2][idx] / (2*sums[2]) + 0.5;
        vals[j] = (float)a;
        vals[j+_numPoints] = (float)b;
        vals[j+2*_numPoints] = (float)c;
    }   // end for

    return valsMat;
}   // end extract


/*
cv::Mat_<byte> CircleDiff::visFeature( const cv::Mat_<float>& fv, const cv::Size& sz) const
{
    assert( fv.total() == 3*_numPoints);
    cv::Mat_<float> f = fv;
    if ( f.cols == 1)   // Ensure row vector
        f = fv.t();
    const float* vals = f.ptr<float>();

    const int hcols = sz.width/2-1;
    const int hrows = sz.height/2-1;
    const double rowRad = hrows;
    const double colRad = hcols;
    cv::Mat_<byte> img = cv::Mat_<byte>::zeros( sz);
    cv::Point p1, p2;
    for ( int i = 0; i < _numPoints; ++i)
    {
        // Outer 
        const cv::Point2d& p1_0 = _points[0][i];
        const cv::Point2d& p2_0 = _points[0][(i+1) % _numPoints];
        p1.x = p1_0.x * colRad + hcols;
        p1.y = p1_0.y * rowRad + hrows;
        p2.x = p2_0.x * colRad + hcols;
        p2.y = p2_0.y * rowRad + hrows;
        float col = 255. * vals[i];
        cv::line( img, p1, p2, cv::Scalar( col), 1);

        // Middle
        const cv::Point2d& p1_1 = _points[1][i];
        const cv::Point2d& p2_1 = _points[1][(i+1) % _numPoints];
        p1.x = p1_1.x * colRad + hcols;
        p1.y = p1_1.y * rowRad + hrows;
        p2.x = p2_1.x * colRad + hcols;
        p2.y = p2_1.y * rowRad + hrows;
        col = 255. * vals[i+_numPoints];
        cv::line( img, p1, p2, cv::Scalar( col), 1);

        // Inner
        const cv::Point2d& p1_2 = _points[2][i];
        const cv::Point2d& p2_2 = _points[2][(i+1) % _numPoints];
        p1.x = p1_2.x * colRad + hcols;
        p1.y = p1_2.y * rowRad + hrows;
        p2.x = p2_2.x * colRad + hcols;
        p2.y = p2_2.y * rowRad + hrows;
        col = 255. * vals[i+2*_numPoints];
        cv::line( img, p1, p2, cv::Scalar( col), 1);
    }   // end for

    return img;
}   // end visFeature
*/
