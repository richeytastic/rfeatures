/*
 * Computes Histograms of Oriented Gradients (HOG) descriptors over an image:
 *
 * The image is tiled into square cells of dimension M (default is 8 pixels high/wide) and
 * each cell is initially assigned a histogram of length N (default 18) with each bin
 * denoting a discretised orientation.
 *
 * The image is iterated over using horizontal and vertical point derivative convolution
 * masks and the orientation angle and strength of the pixel is found. The pixel orientation
 * magnitude is used as a vote which is bilinearly interpolated between both the orientation
 * bins of a cell and across the four cells that overlap the cell sized region centred on
 * the pixel. This interpolation in both orientation and over cell histograms is to reduce
 * the affect of aliasing.
 *
 * Once all pixels have been parsed, a cell's histogram is normalised with respect to the
 * total histogram "energy" in the cells located within a local 2x2 square block of cells.
 * For a given cell, this means that there are a maximum of 4 local blocks that the cell's
 * histogram can contribute to in the normalisation of the other cells in those 4 blocks.
 * This also means that a given cell's histogram is normalised 4 times (i.e. with respect
 * to each of the 4 local blocks it is a part of). The result of this is that each cell is
 * left with 4 histograms (1 for each block), each of length N, where each histogram
 * describes the orientation strengths of the combined pixels in that cell relative to
 * that particular local block of cells.
 *
 * The HOG descriptor is a per cell concatenation of these 4 histograms into single vector.
 *
 * HOG descriptors as implemented here were introduced by Dalal & Triggs in "Histograms
 * of Oriented Gradients for Human Detection" (in proceedings of CVPR 2005). As presented
 * by D&T, the algorithm was empirically tuned for the detection of humans in 2D images
 * using a SVM classifier. Parameters are as used by D&T in this implementation, except
 * for the orientation bins. Orientations are discretised into 18 bins across a whole
 * circle in this implementation. D&T did not distinguish between orientation directions
 * in their work because they applied HOG descriptors to the problem of human detection
 * using a SVM and they reasoned that orientation direction invariance would be preferable
 * for the detection of such objects. However, in the same work, they also mentioned that
 * using the full orientation range for detection of objects other than humans can be of
 * significant benefit (though they did not say how much of a benefit).
 * 
 * Note that pixel orientations cannot be calculated over the first/last row/col of the
 * provided image using pixel centred derivative masks (<-1,0,1> (horizontal), and
 * <-1,0,1>^T (vertical)) so the orientation values at these extremes are calculated
 * using the shorter convolution mask of <-1,1> for horizontal image boundary pixels
 * and its transpose for vertical image boundary pixels. Additionally, because
 * normalisation of cell histograms is with respect to the local block, HOG descriptors
 * for cells at the edge of the image must be artificially increased to full size since
 * each cell only has 2 other 2x2 cell blocks available to it at the edge of an image and
 * corner cells can only be normalised with respect to a single 2x2 cell block. For this
 * reason, the returned HOG descriptors around the edge of the HOG image will be less
 * reliable than HOG descriptors generated from the main part of the image (where each
 * cell can be normalised with respect to 4 2x2 cell blocks). As a result, HOG images
 * should be calculated for an image larger than it needs to be so the boundary HOGs
 * can be discarded.
 *
 * Square-root gamma compression is NOT performed to help improve tolerance to
 * lighting issues in the image - the image must first be processed to do this.
 * Down-weighting of the blocks by a Gaussian spatial window prior to accumulating
 * orientation votes is also not implemented because the resulting performance gain
 * as measured by D&T was found to be only slight.
 *
 * Richard Palmer
 * Curtin University
 * February 2012
 */

#pragma once
#ifndef RFEATURES_HOG_H
#define RFEATURES_HOG_H

#include <boost/foreach.hpp>
#include "FeatureUtils.h"
#include "FeatureOperator.h"
#include "FeatureExceptions.h"
#include "IntegralImage.h"
using RFeatures::ImageTypeException;
using RFeatures::ImageSizeException;
using RFeatures::IntegralImage;


namespace RFeatures
{

class rFeatures_EXPORT HOG : public RFeatures::FeatureOperator
{
public:
    // Construct a version of the provided HOG descriptor image for visualisation purposes
    // only. The orientation histograms of each cell are averaged to provide the average
    // orientation strengths of the pixels in the cell with respect to all the blocks the
    // cell is contained by (i.e. information is lost in this visual representation because
    // histogram information is no longer available concerning orientation strengths in
    // relation to the placement of the cell inside each of its parent blocks). As noted
    // above in the class description, the descriptors for the cells at the boundary of the
    // image cannot be normalised as completely as cells located further inside the image
    // because boundary cells do not contribute to as many local blocks. These cells are
    // included in the returned image however so that the returned image is the same size
    // as the provided HOG image.
    //static cv::Mat createVisualisation( const cv::Mat &hogs, int cellSz=8);

    // img: size must be a whole multiple of cellSize and greater than 2*cellSize.
    // fixedImgSz: must be a whole multiple of cellSize and greater than 2*cellSize.
    // cellSize: width/height in pixels of a local image patch that contributes to an orientation histogram
    // nbins: number of bins to discretise pixel gradients into (default over whole circle, but can be changed with dirDep)
    HOG( const cv::Mat &img, cv::Size fixedImgSz, int cellSize=8, int nbins=9, bool dirDep=true)
                                        throw (ImageTypeException, ImageSizeException);

    virtual ~HOG(){}

protected:
    // Return the per cell HOG descriptors that overlap the specified rectangular image
    // pixel region. Each HOG descriptor is 4 * binsLen long. On return, parameters row,
    // col, width and height will be set to the actual image region that the returned
    // descriptor is applicable to (because each HOG descriptor represents a
    // cellDim x cellDim pixel area). HOG descriptors are not available around the edge
    // of the image (the strip along the top and bottom of the image with
    // width/height = cellDim) because these cells cannot be properly normalised with
    // respect to the local blocks in which these cells reside (because some blocks are
    // off image). Therefore in these cases, the returned region will only contain HOGs
    // for the inner cells.
    virtual cv::Mat_<float> extract( const cv::Rect& subRegion) const;


private:
    const int _nbins;       // Number of bins to discretise cell orientations into
    const cv::Size _fimgSz; // The fixed image size that all regions must be scaled to prior to feature extraction (to ensure same FV lengths)
    const int _cellSz;      // Width/height of a cell in pixels
    const bool _dirDep;     // Bin gradients over a full (true) or a half circle
    const cv::Mat _img;     // The image to extract features on

    double (*calcHgrad)(const cv::Mat &img, int row, int col, int ch);
    double (*calcVgrad)(const cv::Mat &img, int row, int col, int ch);
};  // end class

}   // end namespace

#endif
