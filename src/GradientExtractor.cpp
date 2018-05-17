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

#include <GradientExtractor.h>
#include <FeatureUtils.h>
#include <Convert.h>    // rlib
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <sstream>
#include <cassert>
#include <cstring>
#include <cmath>
using RFeatures::GradientExtractor;
using RFeatures::FeatureExtractor;

namespace {

void makeGradientMaps( const std::vector<cv::Mat_<double> >& hmaps,
                       const std::vector<cv::Mat_<double> >& vmaps,
                       double binRads,
                       std::vector<cv::Mat_<double> >& binGradMaps)
{
    const int nbins = (int)binGradMaps.size();
    int nrows = hmaps[0].rows;
    int ncols = hmaps[0].cols;
    const int nc = ncols*nrows; // ALL maps are continuous so can treat as 1D arrays

    // Continuous maps for the gradient channels and the sum channel
    std::vector<double*> gradRows(nbins);
    for ( int i = 0; i < nbins; ++i)    // Zero out the gradient maps
    {
        binGradMaps[i] = cv::Mat_<double>::zeros( nrows, ncols);
        gradRows[i] = binGradMaps[i].ptr<double>();
    }   // end for

    double mag, magmax, hmax, vmax, hsum, vsum;
    double theta, thetaDiv;
    // Bins and proportions:
    double rprop, lprop;
    int lbin, cbin, rbin;

    const int nch = (int)hmaps.size();   // == vmaps.size()
    std::vector<const double*> hrows(nch);  // Rows for the different horizontal channels
    std::vector<const double*> vrows(nch);  // Rows for the different vertical channels
    for ( int k = 0; k < nch; ++k)
    {
        hrows[k] = hmaps[k].ptr<double>();
        vrows[k] = vmaps[k].ptr<double>();
    }   // end for

    for ( int j = 0; j < nc; ++j)
    {
        // Find the largest magnitude change in contrast over the image channels
        hsum = hmax = hrows[0][j];
        vsum = vmax = vrows[0][j];
        magmax = hsum*hsum + vsum*vsum;
        // Check the remaining channels to find the one with the largest magnitude gradient change
        for ( int k = 1; k < nch; ++k)
        {
            hsum = hrows[k][j];
            vsum = vrows[k][j];
            mag = hsum*hsum + vsum*vsum;
            if ( mag > magmax)
            {
                magmax = mag;
                hmax = hsum;
                vmax = vsum;
            }   // end if
        }   // end for

        mag = sqrt(magmax);
        theta = atan2( vmax, hmax) + CV_PI; // theta in [0,2pi]
        thetaDiv = theta/binRads;
        cbin = int(thetaDiv) % nbins;       // Bin index
        rbin = (cbin+1) % nbins;            // Right bin index
        lbin = (cbin+nbins-1) % nbins;      // Left bin index
        rprop = std::max<double>(thetaDiv - cbin, 0);  // Proportion of magnitude to go into right bin
        lprop = std::max<double>(1.0 - rprop, 0); // Proportion of magnitude to go into left bin (ensure zero bounded)

        assert( lbin >= 0 && lbin < nbins);
        assert( cbin >= 0 && cbin < nbins);
        assert( rbin >= 0 && rbin < nbins);
        assert( mag >= 0);
        assert( rprop*mag >= 0);
        assert( lprop*mag >= 0);
        assert( lbin >= 0 && lbin < nbins);
        assert( cbin >= 0 && cbin < nbins);
        assert( rbin >= 0 && rbin < nbins);

        // Soft bin the magnitudes
        gradRows[lbin][j] += lprop*mag;
        gradRows[cbin][j] += mag;
        gradRows[rbin][j] += rprop*mag;
    }   // end for
}   // end makeGradientMaps


// Make maps of change in horizontal and vertical directions
void makeDirectionChangeMaps( cv::Mat img, std::vector<cv::Mat_<double> >& hmaps, std::vector<cv::Mat_<double> >& vmaps)
{
    const int nch = img.channels();
    std::vector<cv::Mat> imgch;
    cv::split( img, imgch);
    hmaps.resize( nch);   // Horizontal change maps over the image channels
    vmaps.resize( nch);   // Vertical change maps over the image channels
    for ( int i = 0; i < nch; ++i)
    {
        cv::Mat hmap, vmap;
        RFeatures::createChangeMaps( imgch[i], hmap, vmap, false/*don't use absolute value*/);
        hmap.convertTo( hmaps[i], CV_64F);
        vmap.convertTo( vmaps[i], CV_64F);
        assert( hmaps[i].isContinuous());
        assert( vmaps[i].isContinuous());
    }   // end for
}   // end makeDirectionChangeMaps

}   // end namespace


GradientExtractor::GradientExtractor( int nbins, bool gradDirSensitive, int bdims, cv::Mat img)
    : FeatureExtractor(img.size()), _nbins(nbins), _blockDims(bdims), _binRads(0)
{
    setGradientBinRange( gradDirSensitive);

    if ( img.type() == CV_8UC3)
        img = RFeatures::sqrtGammaCorrect(img);

    cv::Mat fimg;
    if ( img.depth() == CV_32F)
        fimg = img;
    else if ( img.depth() == CV_8U)
        img.convertTo( fimg, CV_32F, 1./255, 0);

    std::vector<cv::Mat_<double> > hmaps, vmaps;
    makeDirectionChangeMaps( fimg, hmaps, vmaps);

    std::vector<cv::Mat_<double> > binGradMaps( nbins);
    makeGradientMaps( hmaps, vmaps, _binRads, binGradMaps);

    // Spatially smooth the gradient maps and make integral images of each of the gradient bins
    _gradIntImg.resize(nbins);
    cv::Mat_<double> sumsMap = cv::Mat_<double>::zeros( img.size());
    for ( int i = 0; i < nbins; ++i)
    {
        cv::GaussianBlur( binGradMaps[i], binGradMaps[i], cv::Size(3,3), 0, 0);
        sumsMap += binGradMaps[i];
        cv::integral( binGradMaps[i], _gradIntImg[i], CV_64F);    // Integral image of gradient sums
    }   // end for
    cv::integral( sumsMap, _gradSumIntImg, CV_64F);
}   // end ctor



GradientExtractor::GradientExtractor()
    : FeatureExtractor(), _nbins(0), _binRads(0)
{}   // end ctor



void GradientExtractor::getValidImageTypes( vector<ImageType>& vimgTypes) const
{
    vimgTypes.push_back(BGR);   // Triple channel
    vimgTypes.push_back(Depth);
    vimgTypes.push_back(CIELab);    // Triple channel
    vimgTypes.push_back(EDT);
    vimgTypes.push_back(Grey);
    vimgTypes.push_back(Light);
}   // end getValidImageTypes


std::string GradientExtractor::getParams() const
{
    const bool dirDep = (_nbins * _binRads) > CV_PI;
    std::ostringstream oss;
    oss << _nbins << " " << rlib::asString( dirDep) << " " << _blockDims;
    return oss.str();
}   // end getParams


// protected
FeatureExtractor::Ptr GradientExtractor::createFromParams( const std::string& params) const
{
    // No restrictions on image type
    int nbins;
    int bdims;
    bool dirDep;
    try
    {
        std::istringstream iss(params);
        iss >> nbins;
        dirDep = rlib::asBool(iss);
        iss >> bdims;
    }   // end try
    catch ( const std::exception&)
    {
        throw ExtractorTypeException( "Couldn't read GradientExtractor params from string: " + params);
    }   // end catch

    GradientExtractor *fx = new GradientExtractor();
    fx->_nbins = nbins;
    fx->_blockDims = bdims;
    fx->setGradientBinRange(dirDep);
    return FeatureExtractor::Ptr( fx);
}   // end createFromParams


// protected virtual
FeatureExtractor::Ptr GradientExtractor::initExtractor( const cv::Mat img) const
{
    return FeatureExtractor::Ptr( new GradientExtractor( _nbins, (_nbins * _binRads) > CV_PI, _blockDims, img));
}   // end initExtractor


// private
void GradientExtractor::setGradientBinRange( bool dirSensitive)
{
    _binRads = CV_PI/_nbins;
    if ( dirSensitive)
        _binRads *= 2;
}   // end setGradientBinRange



#define NBINS_MULT 4

// public
cv::Size GradientExtractor::getFeatureDims() const { return cv::Size( _blockDims*_blockDims*NBINS_MULT* _nbins, 1);}   // end getFeatureDims
cv::Size GradientExtractor::getMinSamplingDims() const { return cv::Size( _blockDims+1, _blockDims+1); }   // end getMinSamplingDims


// private
void GradientExtractor::extractHistograms( const cv::Rect& rct, cv::Mat_<double>& cellfvs, double* bsums) const
{
    // Get the number of cells high and wide that we want to features for. Blocks are 2x2 cells.
    const int cdims = _blockDims+1;
    const int ci = rct.height / cdims; // base cell height
    const int cj = rct.width / cdims;  // base cell width
    const int ri = rct.height % cdims; // remaining height
    const int rj = rct.width % cdims;  // remaining width

    const int nbins = _nbins;
    cellfvs.create( cdims*cdims, nbins); // Each row has a nbin histogram

    cv::Rect block( rct.x, rct.y, 0, 0);
    cv::Rect cell( rct.x, rct.y, 0, 0);

    int bk = 0; // Block counter
    int ck = 0; // Cell counter
    for ( int i = 0; i < cdims; ++i)
    {
        block.y = cell.y;
        cell.y += cell.height;
        cell.height = ci + (i < ri ? 1 : 0);
        block.height = cell.y + cell.height - block.y;

        cell.x = rct.x;
        cell.width = 0;
        for ( int j = 0; j < cdims; ++j)
        {
            block.x = cell.x;
            cell.x += cell.width;
            cell.width = cj + (j < rj ? 1 : 0);
            block.width = cell.x + cell.width - block.x;

            double *cellHist = cellfvs.ptr<double>(ck++);
            for ( int k = 0; k < nbins; ++k)
                cellHist[k] = std::max<double>( RFeatures::getIntegralImageSum<double>( _gradIntImg[k], cell), 0);

            if ( i > 0 && j > 0)
                bsums[bk++] = std::max<double>( RFeatures::getIntegralImageSum<double>( _gradSumIntImg, block), 0);
        }   // end for
    }   // end for
}   // end extractHistograms


#define EPS 1e-8
// private
cv::Mat_<float> GradientExtractor::normaliseCells( const cv::Mat_<double>& cellfvs, const double* bsums) const
{
    const int nbs = _nbins;
    assert(cellfvs.cols == nbs);
    const int bdims = _blockDims;
    const int nblocks = bdims*bdims;
    const int cdims = bdims+1;

    const double* histptr = cellfvs.ptr<double>();
    cv::Mat_<float> fv( 1, nblocks*NBINS_MULT*nbs);
    float* fvptr = fv.ptr<float>();

    int c0, c1, c2, c3; // Cell indices for block i
    const double *ch0, *ch1, *ch2, *ch3;    // Pointers to cell histograms for block i
    double bsum;

    const int cellHistsChunk = NBINS_MULT * nbs;

    for ( int i = 0; i < nblocks; ++i)
    {
        // Get the indices of the cell histograms for block i
        c0 = i + (i/bdims);    // integer division
        c1 = c0+1;
        c2 = c0 + cdims;
        c3 = c2+1;
        // The descriptor for this block is the concatenated cell histograms normalised by the block value.
        // |c0|c1|
        // |c2|c3|
        ch0 = &histptr[c0*nbs];
        ch1 = &histptr[c1*nbs];
        ch2 = &histptr[c2*nbs];
        ch3 = &histptr[c3*nbs];

        bsum = bsums[i] + EPS;    // Divisor for L1-sqrt normalisation

        const int k0 = i * cellHistsChunk;
        const int k1 = k0 + nbs;
        const int k2 = k0 + 2*nbs;
        const int k3 = k0 + 3*nbs;
        for ( int j = 0; j < nbs; ++j)
        {
            fvptr[k0+j] = float(sqrt(ch0[j] / bsum));
            fvptr[k1+j] = float(sqrt(ch1[j] / bsum));
            fvptr[k2+j] = float(sqrt(ch2[j] / bsum));
            fvptr[k3+j] = float(sqrt(ch3[j] / bsum));
        }   // end for
    }   // end for

    return fv;
}   // end normaliseCells


// protected virtual
cv::Mat_<float> GradientExtractor::extractFV( const cv::Rect rct) const
{
    cv::Mat_<double> cellfvs;
    double* bsums = (double*)cv::fastMalloc( _blockDims*_blockDims*sizeof(double));
    extractHistograms( rct, cellfvs, bsums);
    const cv::Mat_<float> fv = normaliseCells( cellfvs, bsums);
    cv::fastFree(bsums);
    return fv;
}   // end extractFV
