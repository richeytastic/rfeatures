#include "HOG.h"
using RFeatures::HOG;

#include <cassert>
#include <iostream>
using std::cerr;
using std::endl;
#include <sstream>
using std::ostringstream;
#include <algorithm>
// Version 2.01
// 26 May 2012 - Modified to use cv::Mat (instead of Eigen types) for HOG representation
//             - INTERFACE HAS CHANGED
// 19 June 2012 - Removed custom integral image code and replaced with separate class.
// 10 September 2012 - Removed absolute value bug in selection of best gradient.
//                   - Updated FeatureUtils interface and introduced pointer to functions
//                   - for gradient calculations.
// 17 April 2015 - Finally added in sub rect extraction - use FeatureOperator interface from now on.



double calcHgrad1( const cv::Mat &img, int row, int col, int ch)
{
    return RFeatures::calcHorizontalGrad<cv::Vec<uchar,1> >( img, row, col, ch);
}   // end calcHgrad1


double calcHgrad3( const cv::Mat &img, int row, int col, int ch)
{
    return RFeatures::calcHorizontalGrad<cv::Vec<uchar,3> >( img, row, col, ch);
}   // end calcHgrad3


double calcVgrad1( const cv::Mat &img, int row, int col, int ch)
{
    return RFeatures::calcVerticalGrad<cv::Vec<uchar,1> >( img, row, col, ch);
}   // end calcVgrad1


double calcVgrad3( const cv::Mat &img, int row, int col, int ch)
{
    return RFeatures::calcVerticalGrad<cv::Vec<uchar,3> >( img, row, col, ch);
}   // end calcVgrad3


/*
cv::Mat HOG::createVisualisation( const cv::Mat &hogs, int cellSz)
{
    assert( cellSz % 2 == 0);
    const int halfCellSz = cellSz/2;
    const int HC = hogs.channels();    // HOG channels (nbins * 4)
    const int nbins = HC/4;
    const double GREYMULT = 255./4;

    cv::Mat hogImg( hogs.rows * cellSz, hogs.cols * cellSz, CV_8UC1, cv::Scalar(0));

    // Each input row
    for ( int cellRow = 0; cellRow < hogs.rows; ++cellRow)
    {
        double row = cellRow * cellSz + halfCellSz - 0.5;  // Pixel row index for cell centre
        const double *hogRow = hogs.ptr<double>(cellRow);   // Src

        // Each input column
        for ( int cellCol = 0; cellCol < hogs.cols; ++cellCol)
        {
            double col = cellCol * cellSz + halfCellSz - 0.5;    // Pixel column index for centre of cell

            // Each input orientation channel
            for ( int bin = 0; bin < nbins; ++bin)
            {
                // Calc the angle in radians for this bin. We want to show the direction
                // of the edge (which is orthogonal to the gradient) so we add pi/2.
                double rads = (double)bin / nbins * 2*CV_PI + CV_PI/2;
                if ( rads >= 2*CV_PI) // Ensure rads within range [0,2pi)
                    rads -= 2*CV_PI;

                // Take average of the orientation magnitudes from each of the normalised
                // blocks and convert to a grey value.
                int greyVal = (int)((hogRow[cellCol*HC + 0*nbins + bin]
                               + hogRow[cellCol*HC + 1*nbins + bin]
                               + hogRow[cellCol*HC + 2*nbins + bin]
                               + hogRow[cellCol*HC + 3*nbins + bin])*GREYMULT + 0.5);
                if ( greyVal == 0)  // Skip to next bin if no magnitude for this edge orientation
                    continue;

                // Now need to get the start point and end point of the line representing
                // this edge. Remember here that values are given from the centre of the
                // cell so need to be translated to account for the current image row, col.
                double x1 = (double)halfCellSz * cos(rads);
                double y1 = (double)halfCellSz * sin(rads);
                double xinc = 2*x1 / cellSz; // Incrementing amount along the line in x
                double yinc = 2*y1 / cellSz; // Incrementing amount along the line in y
                double x2 = col - x1;  // Where x2 is as an image pixel x coordinate
                double y2 = row - y1;  // Where y2 is as an image pixel y coordinate
                // Add the grey value to the pixels along the line. Note that range
                // of values includes endpoints of the line (i=0 and i=cellSz)
                for ( int i = 0; i <= cellSz; ++i)
                {
                    int xpos = (int)(x2 + i * xinc); // Position in x along the line (rounded)
                    int ypos = (int)(y2 + i * yinc); // Position in y along the line (rounded)
                    hogImg.ptr<uchar>(ypos)[xpos] += greyVal;
                }   // end for
            }   // end for
        }   // end for
    }   // end for

    return hogImg;
}   // end createVisualisation
*/



HOG::HOG( const cv::Mat& img, cv::Size fimgSz, int csz, int nbs, bool dirDep) throw (ImageTypeException, ImageSizeException)
    : RFeatures::FeatureOperator( fimgSz), _nbins( std::max<int>(2,nbs)),
        _fimgSz(fimgSz), _cellSz( std::max<int>(2,csz)), _dirDep( dirDep), _img(img)
{
    if ( _img.depth() != CV_8U)
        throw ImageTypeException( "Only unsigned char images allowed!");

    int channels = _img.channels();
    if ( channels == 1)
    {
        calcHgrad = calcHgrad1;
        calcVgrad = calcVgrad1;
    }   // end if
    else if ( channels == 3)
    {
        calcHgrad = calcHgrad3;
        calcVgrad = calcVgrad3;
    }   // end else if
    else
        throw ImageTypeException( "HOG can only parse CV_8UC1 or CV_8UC3 images!");

    if ( _fimgSz.height % _cellSz != 0 || _fimgSz.width % _cellSz != 0)
    {
        ostringstream oss;
        oss << "Fixed image size must be whole multiple of requested cell size!" << endl;
        oss << "Requested cell size = " << csz << endl;
        oss << "Fixed image width,height = " << _fimgSz.width << "," << _fimgSz.height << endl;
        throw ImageSizeException( oss.str());
    }   // end if

    if ( (int)_fimgSz.height <= _cellSz*2 || (int)_fimgSz.width <= _cellSz*2)
        throw ImageSizeException( "Provided image is too small for specified cell size!");
}   // end ctor




// For a pixel at position (x,y) and a cell dimension of d, calculate the 4 pixel indices offset from
// x and y by half d in either direction (x1 to the left of and x2 to the right of x and y1 above and
// y2 below y), and the interpolated proportion for each pixel at the corners of the square made by
// these pixels as p1 (top left), p2 (top right), p3 (bottom left) and p4 (bottom right).
void calcSoftPixelInterpolation( int x, int y, int cellSz,
        int &x1, int &x2, int &y1, int &y2,         // Output pixel indices (left, right, top, bottom)
        double &p1, double &p2, double &p3, double &p4) // Output pixel magnitude proportions
{
    int xd = x/cellSz * cellSz; // Integer division - left cell edge for pixel x,y
    int yd = y/cellSz * cellSz; // Integer division - top cell edge for pixel x,y
    const double don2 = cellSz/2;
    x1 = (int)(x-don2);
    x2 = (int)(x+don2);
    y1 = (int)(y-don2);
    y2 = (int)(y+don2);
    p1 = fabs( (double)((xd-x1) * (yd-y1)) / cellSz );
    p2 = fabs( (double)((xd-x2) * (yd-y1)) / cellSz );
    p3 = fabs( (double)((xd-x1) * (yd-y2)) / cellSz );
    p4 = fabs( (double)((xd-x2) * (yd-y2)) / cellSz );
}   // end calcSoftPixelInterpolation



// For a given angle theta, we interpolate the associated gradient magnitude between the gradient
// bins to the left and right of the selected gradient bin. This function returns the selected
// discretised binId for the given theta plus the proportion of the magnitude allocated to the bins
// to the left and right of the bin (in/out parameters leftProp and rightProp). The proportion of
// the gradient allocated to the returned discretised bin is always 0.5.
// theta is in [0,2pi)
int interpolateBinFactor( bool dirDep, double theta, int nbins, double &leftProp, double &rightProp)
{
    int binId = 0;
    double binRads = 0;
    if ( dirDep)
    {
        binId = (int)(nbins * theta / (2*CV_PI));    // Contrast direction sensitive (over whole circle)
        binRads = 2*CV_PI / nbins;
    }   // end if
    else
    {
        if ( theta >= CV_PI)    // Make contrast direction insensitive
            theta -= CV_PI;
        binId = (int)(nbins * theta / CV_PI);
        binRads = CV_PI / nbins;
    }   // end else

    double lowTheta = theta - binRads;
    double highTheta = theta + binRads;
    double lowBin = binId * binRads;
    double highBin = lowBin + binRads;
    const double range = 2 * binRads;
    leftProp = (lowBin - lowTheta) / range;
    rightProp = (highTheta - highBin) / range;
    assert( binId >= 0 && binId < nbins);
    return binId;
}   // end interpolateBinFactor



// Creates integral image of gradient orientations over fixed cells
IntegralImage<double>::Ptr computeGradients( const cv::Mat& img, int nbins, int cellSz, bool dirDep,
                                             double (*calcHgrad)( const cv::Mat&, int row, int col, int ch),
                                             double (*calcVgrad)( const cv::Mat&, int row, int col, int ch))
{
    const int cols = img.cols;
    const int rows = img.rows;
    const int channels = img.channels();

    // Soft-binned pixel orientation magnitudes for integral image calculation.
    // Image is +1 row/col since this will make the calculation of the sums of arbitrary
    // rectangular patches easier later in function calcIntegral.
    cv::Mat pxBinGrads( rows, cols, CV_64FC(nbins), cv::Scalar(0));

    int x1, x2, y1, y2; // Pixels at corners of cell sized region centred on target pixel
    double p1, p2, p3, p4;  // Proportions for interpolation of gradient magnitudes at corner pixels

    // Don't use cv::filter2D with a kernel because we want to produce separate maps
    for ( int j = 0; j < rows; ++j)
    {
        for ( int i = 0; i < cols; ++i)
        {
            // Dalal & Triggs' choice of gradient is the one with the largest
            // magnitude across all colour channels (where mag = sqrt(hg^2 + vg^2))
            // - absolute value used here instead to make comparison more efficient.
            double hg = calcHgrad( img, j, i, 0);
            double vg = calcVgrad( img, j, i, 0);
            double mag = hg*hg + vg*vg;    // Gradient strength (sqrt not needed here)
            for ( int k = 1; k < channels; ++k)
            {
                // horizontal and vertical gradients for colour channel k
                double hg2 = calcHgrad( img, j, i, k);
                double vg2 = calcVgrad( img, j, i, k);
                double mag2 = hg2*hg2 + vg2*vg2;  // Gradient strength (sqrt not needed here)
                if ( mag2 > mag) // Choose the greatest gradient so far
                {
                    hg = hg2;
                    vg = vg2;
                    mag = mag2;
                }   // end if
            }   // end for - all other colour channels checked

            // Get pixel gradient orientation in [0,2pi)
            const double theta = RFeatures::getTheta( vg, hg, mag);
            // mag will be zero at this point if vg and hg are both zero
            if ( mag > 0.0) // Only if vg or hg are not zero
            {
                double lBinProp = 0.0;  // Proportion of magnitude for bin to the left
                static const double mBinProp = 0.5;
                double rBinProp = 0.0; // Proportion of magnitude for bin to the right

                // Discretise theta into a bin, find the bins to the left and the right, and the
                // proportions of the gradient magnitude that each bin must be incremented by.
                int mBin = interpolateBinFactor( dirDep, theta, nbins, lBinProp, rBinProp);
                int lBin = mBin - 1 < 0 ? nbins - 1 : mBin - 1;
                int rBin = mBin + 1 == (int)nbins ? 0 : mBin + 1;

                // Find the interpolated orientation magnitude proportions across the four
                // pixels at the corners of the cell sized region centred on this pixel:
                // |p1|p2|
                // |p3|p4|
                calcSoftPixelInterpolation( i, j, cellSz, x1, x2, y1, y2, p1, p2, p3, p4);
                // NB Some of the x1,x2,y1,y2 values may be out of bounds!

                if ( x1 >= 0 && x1 < cols)  // LEFT PIXELS
                {
                    // Top left pixel
                    if ( y1 >= 0 && y1 < rows)
                    {
                        pxBinGrads.ptr<double>(y1)[x1*nbins+lBin] += p1 * lBinProp * mag;    // Bin left of main bin
                        pxBinGrads.ptr<double>(y1)[x1*nbins+mBin] += p1 * mBinProp * mag;    // Main bin
                        pxBinGrads.ptr<double>(y1)[x1*nbins+rBin] += p1 * rBinProp * mag;    // Bin right of main bin
                    }   // end if

                    // Bottom left pixel
                    if ( y2 >= 0 && y2 < rows)
                    {
                        pxBinGrads.ptr<double>(y2)[x1*nbins+lBin] += p3 * lBinProp * mag;    // Bin left of main bin
                        pxBinGrads.ptr<double>(y2)[x1*nbins+mBin] += p3 * mBinProp * mag;    // Main bin
                        pxBinGrads.ptr<double>(y2)[x1*nbins+rBin] += p3 * rBinProp * mag;    // Bin right of main bin
                    }   // end if
                }   // end if - left pixels

                if ( x2 >= 0 && x2 < cols)  // RIGHT PIXELS
                {
                    // Top right pixel
                    if ( y1 >= 0 && y1 < rows)
                    {
                        pxBinGrads.ptr<double>(y1)[x2*nbins+lBin] += p2 * lBinProp * mag;    // Bin left of main bin
                        pxBinGrads.ptr<double>(y1)[x2*nbins+mBin] += p2 * mBinProp * mag;    // Main bin
                        pxBinGrads.ptr<double>(y1)[x2*nbins+rBin] += p2 * rBinProp * mag;    // Bin right of main bin
                    }   // end if

                    // Bottom right pixel
                    if ( y2 >= 0 && y2 < rows)
                    {
                        pxBinGrads.ptr<double>(y2)[x2*nbins+lBin] += p4 * lBinProp * mag;    // Bin left of main bin
                        pxBinGrads.ptr<double>(y2)[x2*nbins+mBin] += p4 * mBinProp * mag;    // Main bin
                        pxBinGrads.ptr<double>(y2)[x2*nbins+rBin] += p4 * rBinProp * mag;    // Bin right of main bin
                    }   // end if
                }   // end if - right pixels
            }   // end if - valid orientation magnitude
        }   // end for - image columns
    }   // end for - image rows

    // Create the integral image from the soft-binned orientation magnitudes.
    return IntegralImage<double>::Ptr( new IntegralImage<double>( pxBinGrads));
}   // end computeGradients



// Set cell's indices in a block according to the cell ID in the block.
// Cell IDs are:
// 0 - Top left cell of block
// 1 - Top right cell of block
// 2 - Bottom left cell of block
// 3 - Bottom right cell of block
void calcCellIndices( int &cRow, int &cCol, int bRow, int bCol, int cid)
{
    cRow = bRow + (cid > 1 ? 1 : 0);    // Cell 2 and 3 on bottom row of block of 4 cells
    cCol = bCol + cid % 2; // cid on left (0,2) gives +0, else on right (1,3) gives +1
}   // end calcCellIndices



cv::Mat createHOGs( const IntegralImage<double>::Ptr intImg, int nbins, int cellSz)
{
    const int HC = 4*nbins;    // HOG channels (4 * number of orientation bins)
    static const double EPSILON = 1e-12;

    const int bHigh = intImg->rows / cellSz - 1;
    const int bWide = intImg->cols / cellSz - 1;

    cv::Mat hogs = cv::Mat::zeros( bHigh+1, bWide+1, CV_32FC(HC));
    cv::Mat cSums = cv::Mat::zeros( 2, 2, CV_64FC(nbins));    // Tmp orientation bin sums

#ifndef NDEBUG
    const cv::Rect imgRct( 0, 0, intImg->cols, intImg->rows); // DEBUG
#endif

    // Iterate over block rows
    for ( int bRow = 0; bRow < bHigh; ++bRow)
    {
        // Iterate over block columns
        for ( int bCol = 0; bCol < bWide; ++bCol)
        {
            double bNorm = EPSILON; // L2-norm of histogram energy for a block (sqrt of sum of squares)
            int cRow, cCol; // Cell row and column indices
            cSums = cv::Scalar(0);  // Zero out

            // For normalisation, we need the L2-norm of the block of 4 cell
            // histograms (i.e. sum of squares of these histograms).
            for ( int cid = 0; cid < 4; ++cid)
            {
                calcCellIndices( cRow, cCol, bRow, bCol, cid);
                const int row = cellSz * cRow;    // Pixel row index
                const int col = cellSz * cCol;    // Pixel col index
                const cv::Rect rect( col, row, cellSz, cellSz);
                assert( (imgRct & rect) == rect);
                // Get the orientation magnitude sum over each of the different orientations
                for ( int o = 0; o < nbins; ++o)
                {
                    // Get the sum of the orientation bins in this cell
                    double bSum = (*intImg)( rect, o);
                    cSums.ptr<double>(cRow-bRow)[(cCol-bCol)*nbins+o] = bSum;   // Temp
                    bNorm += bSum * bSum; // For L2-norm
                    //bNorm += bSum;    // For L1-norm
                }   // end for - orientation bins
            }   // end for - cells in block

            // Normalise each cell's histogram with respect to the block norm. Note that each cell
            // of this block is normalised in a different location in the block - we need to keep
            // this location information intact. To do this we normalise the cell orientation values
            // at the location in the cell's histogram corresponding to the position of the cell in
            // the block. Cell histogram offsets are 0, nbins, 2 * nbins and 3 * nbins 
            // corresponding to the top left cell of the block, the top right cell, the bottom left
            // cell and the bottom right cell respectively.

            // Iterate over ALL cells of this block and normalise each at the correct descriptor offset.
            for ( int cid = 0; cid < 4; ++cid)
            {
                calcCellIndices( cRow, cCol, bRow, bCol, cid);
                assert( cRow >= 0 && cRow < hogs.rows);
                assert( (cRow - bRow) >= 0 && (cRow - bRow) < cSums.rows);

                int offset = cid * nbins;  // Offset into cell's HOG descriptor vector
                for ( int o = 0; o < nbins; ++o)
                {
                    // L2-norm normalisation
                    const int csc = (cCol-bCol)*nbins+o;
                    assert( csc >= 0 && csc < cSums.channels() * cSums.cols);
                    double bSumNormed = cSums.ptr<double>(cRow-bRow)[csc] / sqrtf(bNorm);
                    // L1-sqrt normalisation
                    //double bSumNormed = sqrt( cSums.ptr<double>(cRow-bRow)[(cCol-bCol)*nbins+o] / bNorm);
                    // Add the normalised value in the HOG descriptor for this cell at the
                    // offset corresponding to this cell's relative position in the block.
                    const int hogc = cCol*HC + offset + o;
                    assert( hogc >= 0 && hogc < HC * hogs.cols);
                    hogs.ptr<float>(cRow)[hogc] += float(bSumNormed);
                }   // end for - orientation bins
            }   // end for - cells in block

        }   // end for - block columns
    }   // end for - block rows

    return hogs;
}   // end createHOGs



// protected
cv::Mat_<float> HOG::extract( const cv::Rect &subRect) const
{
    cv::Mat subreg = _img(subRect);
    cv::Mat resizedSubImg; // Resize subregion of image to be fimgSz
    cv::resize( subreg, resizedSubImg, _fimgSz);
    const IntegralImage<double>::Ptr iimg = computeGradients( resizedSubImg, _nbins, _cellSz, _dirDep, calcHgrad, calcVgrad);
    const cv::Mat hogs = createHOGs( iimg, _nbins, _cellSz);  // CV_32FC(4*_nbins)
    return RFeatures::toRowVector( hogs);
}   // end extract

