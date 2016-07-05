#include "ProHOG.h"
using RFeatures::ProHOG;
#include <algorithm>
#include <cmath>
#include <cstring>
#include <cassert>

#define PXL_MAX 255


void verifyIntegrity( const cv::Mat &x)
{
    assert( x.depth() == CV_64F);
    const int channels = x.channels();
    const int nc = x.cols * channels;

    double minVal = DBL_MAX;
    double maxVal = -DBL_MAX;

    for ( int i = 0; i < x.rows; ++i)
    {
        const double *rowPtr = x.ptr<double>(i);
        for ( int j = 0; j < nc; ++j)
        {
            const double val = rowPtr[j];
            assert( !cvIsInf(val));
            assert( val >= 0.0 && val <= 1.0);
            if ( val < minVal) minVal = val;
            if ( val > maxVal) maxVal = val;
        }   // end for
    }   // end for

    std::cerr << "ProHOG min, max values over all bins: " << minVal << ", " << maxVal << std::endl;
}   // end verifyIntegrity


ProHOG::Ptr ProHOG::create( const std::vector< cv::Mat_<double> >& pxlGds)
{
    return boost::shared_ptr<ProHOG>( new ProHOG( pxlGds));
}   // end create


ProHOG::ProHOG( const std::vector< cv::Mat_<double> >& pxlGds)  // Includes the sum too
    : FeatureOperator( cv::Size( pxlGds[0].cols-1, pxlGds[0].rows-1)),    // Integral image has +1 rows and cols
      _nbins( pxlGds.size() - 1),
      _pxlGradiis( pxlGds),
      _cellDims( pxlGds[0].cols-1, pxlGds[0].rows-1)
{
}   // end ctor



ProHOG::Ptr ProHOG::create( const cv::Mat &img, int nbs, bool dirDep) throw (ImageTypeException)
{
    return boost::shared_ptr<ProHOG>( new ProHOG( img, nbs, dirDep));
}   // end create


ProHOG::ProHOG( const cv::Mat &img, int nbs, bool dirDep, const cv::Size cellDims) throw (ImageTypeException)
    : FeatureOperator( img.size())
{
    init( img, nbs, dirDep, cellDims);
}   // end ctor

#define EPS 1e-8


void ProHOG::normaliseTargetCell( int y, int x, const double *cell, const cv::Mat &sc, cv::Mat &phogs) const
{
    // Provided y is above target row and x is to left of target column.
    const int x1 = x + 1;   // Target col
    const int x2 = x + 2;   // Right of target col
    const double *r0 = sc.ptr<double>(y);   // Above cell row
    const double *r1 = sc.ptr<double>(y+1); // Cell target row
    const double *r2 = sc.ptr<double>(y+2); // Below cell row
    // |Q|W|E|
    // |A|S|D|  Target cell is S
    // |Z|V|C|
    const double q = r0[x];
    const double w = r0[x1];
    const double e = r0[x2];
    const double a = r1[x];
    const double s = r1[x1];
    const double d = r1[x2];
    const double z = r2[x];
    const double v = r2[x1];
    const double c = r2[x2];

    // Get the block sums in 4 different directions from the target cell (+ EPS to avoid div0 errors)
    double bs1 = w+e+s+d + EPS;
    double bs2 = q+w+a+s + EPS;
    double bs3 = a+s+z+v + EPS;
    double bs4 = s+d+v+c + EPS;

    const int nbins = _nbins;
    // Normalise the target cell with regard to these 4 different block directions.
    // Uses squareroot of L1-norm for normalisation as used by Dalal and Triggs for
    // HOG. L1-norm without squareroot was tested for Pro-HOG and found to be worse.
    const int phogLen = 4*nbins;
    double *phog = &phogs.ptr<double>(y)[x*phogLen];
    const int k0 = 0;
    const int k1 = nbins;
    const int k2 = 2*nbins;
    const int k3 = 3*nbins;
    for ( int i = 0; i < nbins; ++i)
    {
        assert( !cvIsInf(cell[i])); // Check for rounding errors!
        const double binVal = cell[i] + EPS;
        phog[k0+i] = std::min<double>( sqrt( binVal / bs1), 1.0);
        phog[k1+i] = std::min<double>( sqrt( binVal / bs2), 1.0);
        phog[k2+i] = std::min<double>( sqrt( binVal / bs3), 1.0);
        phog[k3+i] = std::min<double>( sqrt( binVal / bs4), 1.0);
    }   // end for
}   // end normaliseTargetCell



cv::Mat ProHOG::createProHOG( const cv::Size cellDims, const cv::Rect r) const throw (ImageSizeException)
{
    cv::Rect rct = r;
    if ( rct.width <= 0 || rct.height <= 0)
    {
        rct.width = _pxlGradiis[0].cols-1;
        rct.height = _pxlGradiis[0].rows-1;
    }   // end if

    const int nbins = _nbins;

    if ( cellDims.width <= 0 || cellDims.height <= 0)
        throw ImageSizeException( "Cannot request zero cell dimensions for Pro-HOG calculation!");
    if ( rct.width >= _pxlGradiis[0].cols || rct.height >= _pxlGradiis[0].rows)
        throw ImageSizeException( "Requested Pro-HOG calculation area outside of image bounds!");
    if ( cellDims.width > rct.width || cellDims.height > rct.height)
        throw ImageSizeException( "Requested cell dims greater than requested Pro-HOG calculation bounds!");

    cv::Mat cells = cv::Mat::zeros( cellDims.height+2, cellDims.width+2, CV_64FC(nbins)); // Non-normalised cell values.
    cv::Mat sCells = cv::Mat::zeros( cells.size(), CV_64FC1); // Sum over orientations for a cell
    cv::Mat phogs = cv::Mat::zeros( cellDims.height, cellDims.width, CV_64FC(4*nbins)); // Returned

    // Set cell pixel height/width and remainder
    const int baseWidth = rct.width / cellDims.width;   // Integer division
    const int baseHeight = rct.height / cellDims.height;    // Integer division
    cv::Rect cell( rct.x - baseWidth, rct.y - baseHeight, baseWidth, baseHeight);
    cv::Size cellRem( rct.width % cellDims.width, rct.height % cellDims.height);
    int targetCell = 0;
    int y, x;   // Target cell number and indices

    for ( int i = 0; i < cellDims.height; ++i)
    {
        cell.y += cell.height;   // Set cell y pixel index
        // Increase height of cell by 1 pixel if necessary
        if ( i + cellRem.height >= cellDims.height)
            cell.height = baseHeight + 1;

        double *cellsRow = cells.ptr<double>(i+1);
        double *sCellsRow = sCells.ptr<double>(i+1);

        cell.width = baseWidth;     // Reset cell width for this row
        cell.x = rct.x-baseWidth;

        for ( int j = 0; j < cellDims.width; ++j)
        {
            cell.x += cell.width;
            // Increase width of cell by 1 pixel if necessary
            if ( j + cellRem.width >= cellDims.width)
                cell.width = baseWidth + 1;

            // Get the sum of orientation magnitudes for this cell
            double* cellGradVals = &cellsRow[(j+1)*nbins];
            for ( int b = 0; b < nbins; ++b)
            {
#ifndef NDEBUG
                const cv::Size reqSz(_pxlGradiis[b].cols-1, _pxlGradiis[b].rows-1);
                const cv::Rect pgRect(0,0,reqSz.width,reqSz.height);
                assert( (pgRect & cell) == cell);
#endif
                cellGradVals[b] = RFeatures::getIntegralImageSum<double>( _pxlGradiis[b], cell);
            }   // end for
            sCellsRow[j+1] = RFeatures::getIntegralImageSum<double>( _pxlGradiis[nbins], cell); // Sum of orientation bins for this cell
        }   // end for
    }   // end for

    // Do cell block normalisations
    for ( int i = 0; i < cellDims.height; ++i)
    {
        double *cellsRow = cells.ptr<double>(i+1);
        for ( int j = 0; j < cellDims.width; ++j)
        {
            normaliseTargetCell( i, j, &cellsRow[(j+1)*nbins], sCells, phogs);
        }   // end for - cell cols
    }   // end for - cell rows

//#ifndef NDEBUG
//    verifyIntegrity( phogs);
//#endif
    return phogs;
}   // end createProHOG



void ProHOG::setCellDims( const cv::Size &cellDims)
{
    _cellDims = cellDims;
}   // end setCellDims



cv::Mat_<float> ProHOG::extract( const cv::Rect &rct) const
{
    const cv::Mat phogs = createProHOG( _cellDims, rct);   // CV_64FC(4*nbins), size is cellDims
    return RFeatures::toRowVectors( phogs); // 4*nbins rows, cellDims.width*cellDims.height cols
}   // end extract



cv::Mat ProHOG::createVisualisation( const cv::Mat &phogs, const cv::Size &imgDims, double binRads)
        throw (ImageSizeException)
{
    cv::Mat pimg = cv::Mat::zeros( imgDims.height, imgDims.width, CV_16UC1); // Grey scale return image

    if ( phogs.depth() != CV_64F)
    {
        std::cerr << "ERROR: Can't create Pro-HOG visualisation: given matrix not of depth CV_64F!" << std::endl;
        return pimg;
    }   // end if

    if ( phogs.channels() < 4)
    {
        std::cerr << "ERROR: Can't create Pro-HOG visualisation: given matrix has < 4 channels!" << std::endl;
        return pimg;
    }   // end if

    const double rowRatio = imgDims.height/imgDims.width;
    if ( imgDims.width < phogs.cols || imgDims.height < phogs.rows)
        throw ImageSizeException( "Cannot scale Pro-HOGs to be an image with dimensions smaller than the Pro-HOG matrix itself!");

    const int _nbins = phogs.channels()/4;   // Normalisation always in 4 directions

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

            const int cidx = j*4*_nbins;
            // Each input orientation channel
            for ( int k = 0; k < _nbins; ++k)
            {
                // Get the orientation value
                double cval = phogRow[cidx + k] + phogRow[cidx + _nbins + k]
                            + phogRow[cidx + 2*_nbins + k] + phogRow[cidx + 3*_nbins + k];
                if ( cval <= EPS) // Continue if no orientation for this bin
                    continue;
                int greyVal = (int)(PXL_MAX * cval/4 + 0.5);

                // Calc the angle in radians for this bin. We want to show the direction
                // of the edge (which is orthogonal to the gradient) so we add pi/2.
                // We also take the centre(-0.5) of the angle to draw along since this is
                // the "middle" of the bin. The -0.37 is there simply for a prettier visual!
                double rads = binRads*((double)k-0.5-0.37) + CV_PI/2;
                // Ensure angle is within allowed range (generally, rangle is either
                // over the full circle for contrast variant features or over half a
                // circle for contrast invariant features.
                if ( rads >= _nbins * binRads) rads -= _nbins * binRads;

                double w = fabsf(cos(rads));
                double h = fabsf(sin(rads));
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


int ProHOG::cols() const
{
    return _pxlGradiis[0].cols;
}   // end cols


int ProHOG::rows() const
{
    return _pxlGradiis[0].rows;
}   // end rows



void ProHOG::init( const cv::Mat &img, int nbs, bool dirDep, const cv::Size cellDims) throw (ImageTypeException)
{
    assert( nbs >= 2);
    assert( cellDims.width >= 1 && cellDims.height >= 1);
    _nbins = nbs;
    _doDirDep = dirDep;
    _cellDims = cellDims;

    using RFeatures::GradientsBuilder;

    const int depth = img.depth();
    std::vector<cv::Mat_<double> > grads;

    if ( img.channels() == 3 && depth == CV_8U)
        GradientsBuilder::calculateGradients<cv::Vec3b>( img, nbs, dirDep, grads);
    else if ( img.channels() == 1)
    {
        if ( depth == CV_8U)
            GradientsBuilder::calculateGradients<cv::Vec<byte, 1> >( img, nbs, dirDep, grads);
        else if ( depth == CV_32F)
            GradientsBuilder::calculateGradients<cv::Vec<float, 1> >( img, nbs, dirDep, grads);
        else if ( depth == CV_64F)
            GradientsBuilder::calculateGradients<cv::Vec<double, 1> >( img, nbs, dirDep, grads);
    }   // end else if
    else
        throw ImageTypeException( "[ERROR] ProHOG::init: Invalid image type!");

    assert( grads.size() == nbs);

    // Do spatial smoothing of the gradients, and create the integral images and the gradient sum channel.
    _pxlGradiis.resize(nbs+1); // Set the gradient sum channel
    cv::Mat_<double> gsum = cv::Mat_<double>::zeros(img.rows, img.cols);
    for ( int i = 0; i < nbs; ++i)
    {
        cv::GaussianBlur( grads[i], grads[i], cv::Size(3,3), 0, 0);  // Smooth
        gsum += grads[i];
        cv::integral( grads[i], _pxlGradiis[i], CV_64F);
    }   // end for
    cv::integral( gsum, _pxlGradiis[nbs], CV_64F);
}   // end init
