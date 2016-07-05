#include "Fish2Rect.h"
using RFeatures::Fish2Rect;
#include <cmath>
#include <algorithm>


cv::Mat Fish2Rect::rectify( const cv::Mat& fishImg, double R, double A, double B)
{
    const int rows = fishImg.rows;
    const int cols = fishImg.cols;
    const int hrows = rows/2;
    const int hcols = cols/2;

    const double R2 = R*R;
    const double R4 = R2*R2;

    cv::Mat_<cv::Vec3b> outImg( fishImg.size());
    for ( int i = 0; i < rows; ++i)
    {
        const double y = i - hrows;

        for ( int j = 0; j < cols; ++j)
        {
            const double x = j - hcols;

            const double d2 = x*x + y*y;
            const double d4 = d2*d2;
            const double C = A*(d2-R2) + B*(d4-R4);
            const int id = int(-y*C + i);
            const int jd = int(-x*C + j);

            const cv::Vec3b col = fishImg.at<cv::Vec3b>( i, j);
            if ( id >= 0 && id < rows && jd >= 0 && jd < cols)
                outImg.at<cv::Vec3b>(id, jd) = col;
        }   // end for - cols
    }   // end for - rows

    // Interpolate where points are black
    for ( int i = 0; i < rows; ++i)
    {
        for ( int j = 0; j < cols; ++j)
        {
            cv::Vec3b& c = outImg.at<cv::Vec3b>(i,j);
            if ( c[0] == 0 && c[1] == 0 && c[2] == 0)
            {
                int mult = 0;
                double b = 0, g = 0, r = 0;
                if ( i > 0)
                {
                    const cv::Vec3b& n = outImg.at<cv::Vec3b>(i-1,j);
                    if ( n[0] > 0 || n[1] > 0 || n[2] > 0)
                    {
                        b += n[0];
                        g += n[1];
                        r += n[2];
                        mult++;
                    }   // end if
                }   // end if

                if ( i < rows - 1)
                {
                    const cv::Vec3b& n = outImg.at<cv::Vec3b>(i+1,j);
                    if ( n[0] > 0 || n[1] > 0 || n[2] > 0)
                    {
                        b += n[0];
                        g += n[1];
                        r += n[2];
                        mult++;
                    }   // end if
                }   // end if

                if ( j > 0)
                {
                    const cv::Vec3b& n = outImg.at<cv::Vec3b>(i,j-1);
                    if ( n[0] > 0 || n[1] > 0 || n[2] > 0)
                    {
                        b += n[0];
                        g += n[1];
                        r += n[2];
                        mult++;
                    }   // end if
                }   // end if

                if ( j < cols - 1)
                {
                    const cv::Vec3b& n = outImg.at<cv::Vec3b>(i,j+1);
                    if ( n[0] > 0 || n[1] > 0 || n[2] > 0)
                    {
                        b += n[0];
                        g += n[1];
                        r += n[2];
                        mult++;
                    }   // end if
                }   // end if

                b /= mult;
                g /= mult;
                r /= mult;

                c[0] = int(b+0.5);
                c[1] = int(g+0.5);
                c[2] = int(r+0.5);
            }   // end if
        }   // end for
    }   // end for

    return outImg;
}   // end rectify



double calcOutSizeRatio( double radius, double focLen)
{
    const double beta = asin( radius / focLen);
    return focLen * tan(beta) / radius;
}   // end calcOutSizeRatio



cv::Mat Fish2Rect::rectify( const cv::Mat& fishImg, double focLen)
{
    const int inrows = fishImg.rows;
    const int incols = fishImg.cols;
    const int hrows = inrows/2;
    const int hcols = incols/2;

    const double focLenSq = focLen*focLen;

    const double outSzRatio = calcOutSizeRatio( std::min(hrows, hcols), focLen);
    cv::Mat_<cv::Vec3b> outImg( int(outSzRatio * inrows), int(outSzRatio * incols));
    const int outRows = outImg.rows;
    const int outCols = outImg.cols;
    const int hOutRows = outRows/2;
    const int hOutCols = outCols/2;

    for ( int i = 0; i < outRows; ++i)
    {
        const double y = i - hOutRows;

        for ( int j = 0; j < outCols; ++j)
        {
            const double x = j - hOutCols;

            const double p = sqrt(x*x + y*y + focLenSq) / focLen;
            const int id = cvRound( y/p + hrows); // Fisheye distorted row
            const int jd = cvRound( x/p + hcols); // Fisheye distorted col

            cv::Vec3b col(0,0,0);
            if ( id >= 0 && id < inrows && jd >= 0 && jd < incols)
                col = fishImg.at<cv::Vec3b>( id, jd);
            outImg.at<cv::Vec3b>(i,j) = col;
        }   // end for - cols
    }   // end for - rows

    cv::Mat dimg;
    cv::resize( outImg, dimg, fishImg.size());
    return dimg;
}   // end rectify
