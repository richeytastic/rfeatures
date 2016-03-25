/*
 * Computes Proportional Histograms of Oriented Gradients (Pro-HOG) descriptors over an image:
 *
 * The image is tiled into cells (not necessarily square) of dimension proportional to the image
 * dimensions. A histogram of length N (default 9) is associated with each cell. Each bin of this
 * histogram represents a discretised orientation and the continuous value of each bin represents
 * that orientation's magnitude.
 *
 * Horizontal and vertical point derivative convolution masks are used to iterate over every
 * pixel in the image and the orientation and strength of the pixel is calculated. The pixel
 * orientation strength is used to soft bin between that orientation bin and the two bins either
 * side of it. At this stage, each pixel gets its own set of orientation bins. This is represented
 * by an integral image of the same number of channels as bins. Additionally, all orientation
 * magnitudes are summed over the pixel and placed into a second integral image which will be
 * used during normalisation.
 *
 * Once all pixels have been parsed, the cell dimensions can be defined as a proportion of the
 * image dimensions. Final construction requires pixel histograms to be summed over cell regions.
 * This is accomplished using the first integral image. The sum of each orientation value for
 * a cell is then normalised in four different directions w.r.t the four blocks of 2x2 cells
 * that overlap this cell. Normalisation uses the L1 norm of the block of cells (i.e. the L1 norm
 * of the concatenation of the four cell histograms) for each orientation bin in the target
 * cell's histogram. The four cell normalisations are then concatenated into a 4*N vector for
 * that cell. Finally, the dominant direction of the Pro-HOG descriptor is found and the vector
 * shifted so that the most dominant direction is at the initial index (helps with invariance
 * to rotation).
 *
 * The original HOG descriptor was introduced by Dalal & Triggs in "Histograms of Oriented
 * Gradients for Human Detection" (in proceedings of CVPR 2005). As presented by D&T, the
 * algorithm was empirically tuned for the detection of humans in 2D images using a SVM
 * classifier.
 * 
 * Note that pixel orientations cannot be calculated over the first/last row/col of the image
 * using pixel centred derivative masks (<-1,0,1> (horizontal), and <-1,0,1>^T (vertical)) so
 * the orientation values at these extremes are calculated using the shorter convolution masks
 * of <-1,1> (horizontal), and <-1,1>^T (vertical). Additionally, because each cell is created
 * to include information about the cells around it, Pro-HOGs at the edges of the image will be
 * less reliable than the Pro-HOGs further inside the image.
 *
 * As part of the initial processing of the image to find the pixel first order derivatives,
 * square-root gamma compression is performed to help improve tolerance to lighting issues.
 *
 * Richard Palmer
 * Curtin University 2012
 */

#pragma once
#ifndef RFEATURES_PROHOG_H
#define RFEATURES_PROHOG_H

#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>
#include "FeatureUtils.h"
#include "GradientsBuilder.h"
#include "FeatureOperator.h"
using RFeatures::FeatureOperator;
#include "FeatureExceptions.h"
using RFeatures::ImageTypeException;
using RFeatures::ImageSizeException;


namespace RFeatures
{

class rFeatures_EXPORT ProHOG : public FeatureOperator
{
public:
    typedef boost::shared_ptr<ProHOG> Ptr;

    // Create a Pro-HOG feature extractor from a pre-computed integral image of pixel
    // gradients (see GradientsBuilder). As well as bi-linear interpolation of gradient
    // magnitudes in the pixel histograms, gradient magnitudes are soft-binned into
    // adjacent vertical and horizontal pixels.
    // The number of bins is found as the number of channels in pxlGradients minus 1.
    // The last channel N+1 is the sum of the gradients over the first N channels (to
    // allow for efficient normalisation).
    static Ptr create( const std::vector< cv::Mat_<double> >& pxlGradients);
    ProHOG( const std::vector< cv::Mat_<double> >& pxlGradients);

    // Create a Pro-HOG feature vector from the provided image (may want to so sqrt gamma correction beforehand).
    // nbins: the number of bins to discretise pixel gradients into over a whole circle.
    // I.e. for N bins, there are 360/N degrees per bin (with default direction dependent contrast).
    // dirDep: if true (default), orientations depend on direction. If false, contrast direction
    // is ignored.
    static Ptr create( const cv::Mat& img, int nbins=9, bool dirDep=true) throw (ImageTypeException);
    ProHOG( const cv::Mat &img, int nbins=9, bool dirDep=true, const cv::Size cellDims=cv::Size(1,1)) throw (ImageTypeException);

    virtual ~ProHOG(){}

    // Create and return the Pro-HOG feature vector calculated over the specified region of
    // the image (or the whole image by default). Created Pro-HOG will be cellDims wide/high
    // Throws exception if requested rectangle dimensions are outside of image bounds or
    // if the rectangle width/height is less than the specified number of cells.
    // Type of returned matrix is CV_64FC(4*nbins_)
    cv::Mat createProHOG( const cv::Size cellDims, const cv::Rect r=cv::Rect(0,0,0,0)) const throw (ImageSizeException);

    // Set the cell dimensions to be used when calling operator() (default is 1 cell per pixel)
    void setCellDims( const cv::Size &cellDims);
    const cv::Size& getCellDims() const { return _cellDims;}

    // Create and return a visualisation of the provided Pro-HOG feature vector scaled up
    // to the provided dimensions. These dimensions must be greater than or equal to the
    // size of the provided Pro-HOG image (or exception is thrown).
    // Parameter binRads specifies the number of radians per histogram bin e.g.
    // for 9 bins over 360 this is 9^-1 * 2*pi. For 9 bins over 180 degrees it is 9^-1 * pi.
    static cv::Mat createVisualisation( const cv::Mat &phogs, const cv::Size &imgDims, double perBinRads) throw (ImageSizeException);

    int rows() const;
    int cols() const;

    inline int getNumBins() const { return _nbins;}

protected:
    // Returned vector has 4*nbins rows and celldims.width*celldims.height cols.
    virtual cv::Mat_<float> extract( const cv::Rect &rct) const;

private:
    int _nbins;                       // Number of orientation bins (N)
    bool _doDirDep;
    std::vector< cv::Mat_<double> > _pxlGradiis;  // Per pixel integral image of orientations (N+1 channels)
    cv::Size _cellDims;                         // Default cell dims for operator()

    void calcPixelGradients( const cv::Mat &img);    // Creates integral image of pixel gradients
    void normaliseTargetCell( int y, int x, const double *cell, const cv::Mat &sumCells, cv::Mat &phogs) const;

    // Ctor helper
    void init( const cv::Mat &img, int nbins, bool dirDep, const cv::Size cellDims) throw (ImageTypeException);
};  // end class

}   // end namespace

#endif
