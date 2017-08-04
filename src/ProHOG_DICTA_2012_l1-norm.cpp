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

#include "ProHOG.h"
using RFeatures::ProHOG;
#include "FeatureUtils.h"
using RFeatures::getTheta;
using RFeatures::calcHorizontalGrad;
using RFeatures::calcVerticalGrad;
using RFeatures::sqrtGammaCorrect;

#include <cmath>
#include <cstring>
#include <cassert>

#include <iostream>
using std::cerr;
using std::endl;

typedef unsigned char uchar;
typedef unsigned int uint;

#define PXL_MAX 255
#define EPS 1e-10



ProHOG::ProHOG( const cv::Mat &img, int nbs) throw (ImageTypeException)
    : nbins( nbs < 2 ? 2 : nbs), pxlIntImg( img.size(), nbins), binIntImg( img.size(), 1),
      BIN_ANGLE( 2*M_PI / nbins)
{
    if ( img.depth() != CV_8U)
        throw ImageTypeException( "Image is not of depth CV_8U!");
    calcPixelGradients( img);
}   // end ctor



void ProHOG::calcTargetCell( int y, int x, const double *cell, const cv::Mat &sc, cv::Mat &phogs) const
{
    const int nbins2 = 2 *nbins;
    const int nbins3 = 3 *nbins;

    // Get the block sums in the 4 different diagonal directions from the target cell
    // Provided y is above target row and x is to left of target column.
    const int y1 = y + 1;   // Target row
    const int y2 = y + 2;   // Below target row
    const int x1 = x + 1;   // Target col
    const int x2 = x + 2;   // Right of target col
    // |bs1|bs2|
    // |bs3|bs4|
    double bs1 = sc.at<double>(y,x) + sc.at<double>(y,x1) + sc.at<double>(y1,x) + sc.at<double>(y1,x1) + EPS;
    double bs2 = sc.at<double>(y,x1) + sc.at<double>(y,x2) + sc.at<double>(y1,x1) + sc.at<double>(y1,x2) + EPS;
    double bs3 = sc.at<double>(y1,x) + sc.at<double>(y1,x1) + sc.at<double>(y2,x) + sc.at<double>(y2,x1) + EPS;
    double bs4 = sc.at<double>(y1,x1) + sc.at<double>(y1,x2) + sc.at<double>(y2,x1) + sc.at<double>(y2,x2) + EPS;

    // Normalise the target cell with regard to these 4 different block directions.
    // Uses L1-norm normalisation. Found to be more effective than L1-sqrt
    // normalisation as used by Dalal and Triggs.
    double *phog = &phogs.ptr<double>(y)[x*nbins*4];
    double binVal = 0;
    for ( int k = 0; k < nbins; ++k)
    {
        binVal = cell[k];   // Check for rounding errors!
        if ( binVal < EPS) binVal = 0;
        phog[k] = binVal/bs1;
        phog[nbins+k] = binVal/bs2;
        phog[nbins2+k] = binVal/bs3;
        phog[nbins3+k] = binVal/bs4;
    }   // end for
}   // end calcTargetCell



cv::Mat ProHOG::operator()( const cv::Size &numCells, const cv::Rect &r) const throw (ImageSizeException)
{
    cv::Rect rct = r;
    if ( rct.width <= 0 || rct.height <= 0)
    {
        rct.width = pxlIntImg.cols;
        rct.height = pxlIntImg.rows;
    }   // end if

    if ( numCells.width <= 0 || numCells.height <= 0)
        throw ImageSizeException( "Cannot request zero cell dimensions for Pro-HOG calculation!");
    if ( rct.width > pxlIntImg.cols || rct.height > pxlIntImg.rows)
        throw ImageSizeException( "Requested Pro-HOG calculation area outside of image bounds!");
    if ( numCells.width > rct.width || numCells.height > rct.height)
        throw ImageSizeException( "Requested cell dims greater than requested Pro-HOG calculation bounds!");

    // Matrix to contain the non-normalised cell values.
    cv::Mat cells = cv::Mat::zeros( numCells.height+2, numCells.width+2, CV_64FC(nbins));
    // Matrix to contain the sum over orientations for a cell
    cv::Mat sCells = cv::Mat::zeros( numCells.height+2, numCells.width+2, CV_64FC1);
    cv::Mat phogs = cv::Mat::zeros( numCells.height, numCells.width, CV_64FC(4*nbins));   // For return

    // Set cell pixel height/width and remainder
    const int baseWidth = rct.width / numCells.width;   // Integer division
    const int baseHeight = rct.height / numCells.height;    // Integer division
    cv::Rect cell( rct.x-baseWidth, rct.y-baseHeight, baseWidth, baseHeight);
    cv::Size cellRem( rct.width % numCells.width, rct.height % numCells.height);
    int targetCell, y, x;   // Target cell number and indices

    for ( int i = 0; i < numCells.height; ++i)
    {
        cell.y += cell.height;   // Set cell y pixel index
        // Increase height of cell by 1 pixel if necessary
        if ( i + cellRem.height >= numCells.height)
            cell.height = baseHeight + 1;

        double *cellsRow = cells.ptr<double>(i+1);
        double *sCellsRow = sCells.ptr<double>(i+1);

        cell.width = baseWidth;     // Reset cell width for this row
        cell.x = rct.x-baseWidth;

        for ( int j = 0; j < numCells.width; ++j)
        {
            cell.x += cell.width;
            // Increase width of cell by 1 pixel if necessary
            if ( j + cellRem.width >= numCells.width)
                cell.width = baseWidth + 1;

            pxlIntImg( cell, &cellsRow[(j+1)*nbins]); // Get the sum of orientation magnitudes for this cell
            sCellsRow[j+1] = binIntImg( cell, 0);  // Sum of orientation bins for this cell

            // Calculate the target cell indices y,x and calculate if in range
            targetCell = i * phogs.cols + j - cells.cols;
            if ( targetCell >= 0)
            {
                y = targetCell / phogs.cols;   // Row of target cell in phogs
                x = targetCell % phogs.cols;   // Column of target cell in phogs
                calcTargetCell( y, x, &cells.ptr<double>(y+1)[(x+1)*nbins], sCells, phogs);
            }   // end if
        }   // end for
    }   // end for

    // Finish calculating cell normalisations over the remaining cells
    int maxCells = phogs.rows * phogs.cols;
    int i = targetCell+1;
    if ( i < 0) i = 0;
    for ( ; i < maxCells; ++i)
    {
        y = i / phogs.cols;   // Row of target cell in phogs
        x = i % phogs.cols;   // Column of target cell in phogs
        calcTargetCell( y, x, &cells.ptr<double>(y+1)[(x+1)*nbins], sCells, phogs);
    }   // end for

    return phogs;
}   // end operator()



// For a given angle theta, we interpolate the associated gradient magnitude between the gradient
// bins to the left and right of the selected gradient bin. This function returns the selected
// discretised binId for the given theta plus the proportion of the magnitude allocated to the bins
// to the left and right of the bin (in/out parameters leftProp and rightProp). The proportion of
// the gradient allocated to the returned discretised bin is always 0.5.
int ProHOG::softBinGradient( double theta, double &leftProp, double &rightProp)
{
    int binId = (int)(theta / BIN_ANGLE);    // Contrast direction sensitive (over whole circle)
    double lowTheta = theta - BIN_ANGLE;
    double highTheta = theta + BIN_ANGLE;
    double lowBin = binId * BIN_ANGLE;
    double highBin = lowBin + BIN_ANGLE;
    const double range = 2 * BIN_ANGLE;
    leftProp = (lowBin - lowTheta) / range;
    rightProp = (highTheta - highBin) / range;
    return binId;
}   // end softBinGradient



// Calculate the magnitude (returned) and orientation (theta) over all channels
// of simg at given pixel. The pixels are first sqrt gamma corrected and mapped
// to dimg and dimg is presumed to already have sqrt gamma corrected pixels
// above and to the left of the pixel.
double calcGradient( cv::Mat &dimg, const cv::Mat &simg, int row, int col, double &theta)
{
    const int channels = simg.channels();
    const int cidx = col*channels;
    double mag = 0, bmag = 0;
    int hg = 0, bhg = 0, vg = 0, bvg = 0;

    // Gradient choice is the one over all channels with the largest magnitude.
    for ( int k = 0; k < channels; ++k)
    {
        if ( row == 0 && col < simg.cols - 1)    // Ensure first row is gamma corrected
            dimg.ptr<uchar>(row)[cidx+channels+k] = sqrt( simg.ptr<const uchar>(row)[cidx+channels+k] * PXL_MAX);
        if ( row < simg.rows - 1)  // Gamma correct pixels of next row
            dimg.ptr<uchar>(row+1)[cidx+k] = sqrt( simg.ptr<const uchar>(row+1)[cidx+k] * PXL_MAX);

        hg = calcHorizontalGrad( dimg, row, col, k);
        vg = calcVerticalGrad( dimg, row, col, k);
        mag = fabs(hg) + fabs(vg);    // Gradient strength (sqrt not needed here)
        if ( mag > bmag)
        {
            bmag = mag;
            bhg = hg;
            bvg = vg;
        }   // end if
    }   // end for

    // Get pixel gradient orientation in [0,2pi)
    theta = getTheta( bvg, bhg, bmag);
    return bmag;    // Zero returned if bvg and bhg are both zero
}   // end calcGradient



void ProHOG::calcPixelGradients( const cv::Mat &simg)
{
    cv::Mat dimg( simg.size(), simg.type());  // Destination image for copying over gamma corrected pixels
    const int cols = simg.cols;
    const int rows = simg.rows;
    double bins[nbins]; // A pixel's soft binned gradient magnitudes
    const int binsSz = sizeof(double)*nbins;
    static const double mBinProp = 0.5; // Centre bin proportion
    double lBinProp, rBinProp;
    int mBin, lBin, rBin;   // Bin indices for soft binning of gradient magnitude
    double bsum;    // Sum over bins for binIntImg

    // Sqrt gamma correct the first pixel over all channels
    for ( int k = 0; k < nbins; ++k)
        dimg.ptr<uchar>(0)[k] = sqrt( simg.ptr<const uchar>(0)[k] * PXL_MAX);

    double theta, mag;  // Orientation and magnitude for each pixel
    for ( int i = 0; i < rows; ++i)
    {
        for ( int j = 0; j < cols; ++j)
        {
            mag = calcGradient( dimg, simg, i, j, theta);
            if ( mag < EPS) mag = 0;
            memset( bins, 0, binsSz);    // Zero out bin magnitudes array
            bsum = 0;

            if ( mag > 0)
            {
                lBinProp = 0; // Proportion of magnitude for bin to the left
                rBinProp = 0; // Proportion of magnitude for bin to the right

                // Discretise theta into a bin, find the bins to the left and the right, and the
                // proportions of the gradient magnitude that each bin has for this pixel.
                mBin = softBinGradient( theta, lBinProp, rBinProp);
                lBin = mBin - 1 < 0 ? nbins - 1 : mBin - 1;
                rBin = mBin + 1 == nbins ? 0 : mBin + 1;
                bins[mBin] = mBinProp * mag;
                bins[lBin] = lBinProp * mag;
                bins[rBin] = rBinProp * mag;
                bsum = bins[mBin] + bins[lBin] + bins[rBin];
            }   // end if

            pxlIntImg.addValue( bins);
            binIntImg.addValue( &bsum);
        }   // end for - image columns
    }   // end for - image rows
}   // end calcPixelGradients



cv::Mat ProHOG::createVisualisation( const cv::Mat &phogs, const cv::Size &imgDims) const
        throw (ImageSizeException)
{
    if ( phogs.channels() != 4*nbins)
        throw ImageSizeException( "Channel count of provided Pro-HOG map does not match channels set for this Pro-HOG object!");
    return ProHOG::createVisualisation( phogs, imgDims, BIN_ANGLE);
}   // end createVisualisation



cv::Mat ProHOG::createVisualisation( const cv::Mat &phogs, const cv::Size &imgDims, double binAngle)
        throw (ImageSizeException)
{
    if ( imgDims.width < phogs.cols || imgDims.height < phogs.rows)
        throw ImageSizeException( "Cannot scale Pro-HOGs to be an image with dimensions smaller than the Pro-HOG matrix itself!");

    const int nbins = phogs.channels()/4;   // Normalisation always in 4 directions
    cv::Mat pimg = cv::Mat::zeros( imgDims.height, imgDims.width, CV_16UC1); // Grey scale return image

    // Set cell pixel height/width and remainder
    const int baseWidth = imgDims.width / phogs.cols;
    const int baseHeight = imgDims.height / phogs.rows;
    cv::Rect cell( -baseWidth, -baseHeight, baseWidth, baseHeight);
    cv::Size cellRem( imgDims.width % phogs.cols, imgDims.height % phogs.rows);
    const double halfCellWidth0 = (double)baseWidth/2;
    const double halfCellWidth1 = (double)(baseWidth+1)/2;
    const double halfCellHeight0 = (double)baseHeight/2;
    const double halfCellHeight1 = (double)(baseHeight+1)/2;
    double halfCellWidth, halfCellHeight;   // Changes since base cell size may change

    double maxVal = 0;

    // Pixel indices for image - relate to top left corner of each cell
    for ( int i = 0; i < phogs.rows; ++i)
    {
        cell.y += cell.height;
        halfCellHeight = halfCellHeight0;
        // Increase height of cell by 1 pixel if necessary
        if ( i + cellRem.height >= phogs.rows)
        {
            cell.height = baseHeight + 1;
            halfCellHeight = halfCellHeight1;
        }   // end if
        double r = (double)cell.y + halfCellHeight;  // Pixel row for centre of cell

        cell.width = baseWidth;     // Reset cell width for this row
        cell.x = -cell.width;
        const double *phogRow = phogs.ptr<double>(i);   // Source row of cell

        for ( int j = 0; j < phogs.cols; ++j)
        {
            cell.x += cell.width;
            halfCellWidth = halfCellWidth0;
            // Increase width of cell by 1 pixel if necessary
            if ( j + cellRem.width >= phogs.cols)
            {
                cell.width = baseWidth + 1;
                halfCellWidth = halfCellWidth1;
            }   // end if
            double c = (double)cell.x + halfCellWidth;  // Pixel col for centre of cell

            const int cidx = j*4*nbins;
            // Each input orientation channel
            for ( int k = 0; k < nbins; ++k)
            {
                // Get the orientation value
                double cval = phogRow[cidx + k] + phogRow[cidx + nbins + k]
                            + phogRow[cidx + 2*nbins + k] + phogRow[cidx + 3*nbins + k];
                if ( cval <= EPS) // Continue if no orientation for this bin
                    continue;
                int greyVal = (int)(PXL_MAX * cval/4 + 0.5);

                // Calc the angle in radians for this bin. We want to show the direction
                // of the edge (which is orthogonal to the gradient) so we add pi/2.
                // We also take the centre(-0.5) of the angle to draw along since this is
                // the average of the bin.
                double rads = binAngle*((double)k-0.25) + M_PI/2;
                // Ensure angle is within allowed range (generally, rangle is either
                // over the full circle for contrast variant features or over half a
                // circle for contrast invariant features.
                if ( rads >= nbins * binAngle) rads -= nbins * binAngle;

                double w = fabs(cos(rads));
                double h = fabs(sin(rads));
                // Add the value to the pixels along the line.
                for ( int ci = 0; ci < cell.height; ++ci)
                {
                    int ypos = (int)(((double)ci - halfCellHeight) * h + r);
                    short *pimgrow = pimg.ptr<short>(ypos);
                    for ( int cj = 0; cj < cell.width; ++cj)
                    {
                        int xpos = (int)(((double)cj - halfCellWidth) * w + c);
                        pimgrow[xpos] += greyVal;
                        if ( pimgrow[xpos] > maxVal)
                            maxVal = pimgrow[xpos];
                    }   // end for - all cell columns
                }   // end for - all cell rows
            }   // end for - all channels
        }   // end for - all Pro-HOG columns
    }   // end for - all Pro-HOG rows

    // Scale and convert for return
    cv::Mat img;
    double scale = 1;   // Only scale if max value is greater than in displayable range
    if ( maxVal > PXL_MAX)
        scale = (double)PXL_MAX/maxVal;
    pimg.convertTo( img, CV_8U, scale);
    return img;
}   // end createVisualisation
