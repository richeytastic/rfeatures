#pragma once
#ifndef RFEATURES_FEATURE_UTILS_H
#define RFEATURES_FEATURE_UTILS_H

#ifdef _WIN32
// Disable warnings about MSVC compiler not implementing exception specifications
#pragma warning( disable : 4290)
// Disable warnings about standard template library specialisations not being exported in the DLL interface
#pragma warning( disable : 4251)
#pragma warning( disable : 4275)
#endif

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <string>
using std::string;
#include <vector>
using std::vector;
#include <ostream>
using std::ostream;
#include <istream>
using std::istream;
#include <opencv2/opencv.hpp>
#include <boost/foreach.hpp>
typedef unsigned char uchar;
typedef unsigned char byte;
#include "rFeatures_Export.h"
#include "FeatureExceptions.h"
using RFeatures::DescriptorLengthException;
#include "DepthSegmenter.h"
#include <Random.h> // rlib


namespace RFeatures
{

// Convenience class for single channel images whose depth we don't care about.
// Provided image must be continuous.
class rFeatures_EXPORT DepthAgnosticImg
{
public:
    DepthAgnosticImg( const cv::Mat& img);   // img must be single channel, but any depth
    double operator()( const cv::Point& p) const;   // Get the value at p

private:
    const cv::Mat _img;
    const byte* _imgArr;
    const int _rows, _cols, _depth, _elemSz;
    const cv::Rect _imgRct;
};  // end class


// Given integral image X, calculate and return the sum over the given rectangle.
template <typename T>
T getIntegralImageSum( const cv::Mat& iimgX, const cv::Rect& rct);

// Given integral image X and the integral image of its square values
// (calculate using cv::integral(InputArray src, OutputArray sum, OutputArray sqsum))
// calculate the uncorrected variance over the given rectangle. Take sqrt for standard deviation.
// maskii (if given) is the integral image of counts to be used in the calculation which MUST
// have been used previously to create iimgX and iimgXsq! (it cannot be passed arbitrary and a
// correct result be returned).
// maskii should be created using a function such as createMaskIntegralImage (below).
// T is the template parameter of iimgX.
template <typename T>
double calcVariance( const cv::Mat& iimgX, const cv::Mat_<double>& iimgXsq, const cv::Rect& rct, cv::Mat_<int> maskii=cv::Mat_<int>());

template <typename T>
cv::Mat_<int> createMaskIntegralImage( const cv::Mat& X, T minX, T maxX);

// Given a rectangle within bounding dimensions sz0, find and return the proportionately positioned
// and dimensioned rectangle that fits within the dimensions sz1.
rFeatures_EXPORT cv::Rect calcRelativeRect( const cv::Rect& rct0, const cv::Size& sz0, const cv::Size& sz1);

rFeatures_EXPORT cv::Rect createRandomRect( const cv::Size& bounds, const cv::Size& minSz, rlib::Random& rndGen);

// Scale the rectangle by the given factor (returns as well for convenience).
rFeatures_EXPORT cv::Rect& scale( cv::Rect& rect, float factor);

// Get the point that is offset from the top left corner of rct by offset as a proportion of the width
// and height of rct. So, for example, to return the point at the centre of rct, offset should be 0.5, 0.5.
// For the point at the centre bottom, offset should be 0.5, 1.0. And so on. Obviously, offset points outside
// the boundary of rct can be calculated.
rFeatures_EXPORT cv::Point calcPixelOffset( const cv::Rect& rct, const cv::Point2f& offset);

// As above, but returns a floating point value.
rFeatures_EXPORT cv::Point2f calcOffset( const cv::Rect_<float>& rct, const cv::Point2f& offset);

// Calculate the offset into the rotated rectangle from the top left corner of the rectangle
// assuming a coordinate system relative to the angle of the rectangle.
rFeatures_EXPORT cv::Point2f calcOffset( const cv::RotatedRect&, const cv::Point2f& offset);

// Get the point in the middle of the given rectangle (rounded) (special case of calcPixelOffset above).
rFeatures_EXPORT cv::Point calcPixelCentre( const cv::Rect& rct);

// Convenience functions to check if cv::Rect inner is within
// the bounds of the given outer cv::Rect, or the rows and cols of m.
rFeatures_EXPORT bool isWithin( const cv::Rect& outer, const cv::Rect& inner);
rFeatures_EXPORT bool isWithin( const cv::Mat& m, const cv::Rect& inner);
rFeatures_EXPORT bool isWithin( const cv::Mat& m, const cv::Point& inner);
rFeatures_EXPORT bool isContained( const cv::Rect& outer, const cv::Rect& inner);   // Synonymous with isWithin
rFeatures_EXPORT bool isContained( const cv::Mat& m, const cv::Rect& inner);    // Synonymous with isWithin
rFeatures_EXPORT bool isContained( const cv::Mat& m, const cv::Point& inner);    // Synonymous with isWithin

// Given a single channel image of any depth, find the point giving a
// local minimum closest to startPoint between p0 and p1.
rFeatures_EXPORT cv::Point findLocalMin( const cv::Mat& singleChannelImage,
                        const cv::Point& startPoint, const cv::Point& p0, const cv::Point& p1);

// Find the global minimum value in the given image between points p0 and p1.
// Returns the point and sets out param minVal to the value if not null.
rFeatures_EXPORT cv::Point findMin( const cv::Mat_<float>&, const cv::Point& p0, const cv::Point& p1, float* minVal=NULL);

// Treats the provided image as a depth map and finds the maximum difference in depth
// as measured orthogonally to the base vector between points p1 and p0. Returns the depth
// delta itself, and copies the location of this point to dpoint on exit.
rFeatures_EXPORT float findOrthogonalMaxDelta( const cv::Mat_<float>&, const cv::Point& p0, const cv::Point& p1, cv::Point& dpoint);

// Find the mean position of the non-zero pixels of mask.
// Returns true if mask contains at least one non-zero pixel.
rFeatures_EXPORT bool findMeanPosition( cv::Point& p, const cv::Mat_<byte> mask);

// Find and return the first point from p1 to p2 (exclusive) that intersects with a non-zero pixel of m.
// If no non-zero pixel is found on the way to p2, cv::Point(-1,-1) is returned.
rFeatures_EXPORT cv::Point findIntersectionPoint( const cv::Mat_<byte>& m, const cv::Point& p1, const cv::Point& p2);

// As above, but once a boundary pixel is hit, the function only returns once a zero pixel is hit.
rFeatures_EXPORT cv::Point findOuterIntersectionPoint( const cv::Mat_<byte>& m, const cv::Point& p1, const cv::Point& p2);

// All non-zero element indices of img are extracted as Points and placed in provided vector.
// Useful for functions like cv::convexHull which takes a vector of points as input.
// Returns the number of points appended to pts.
rFeatures_EXPORT int nonZeroToPoints( const cv::Mat_<byte>& img, std::vector<cv::Point>& pts);

// Set boundingBox to be the rectangle of minimum area that completely encloses the
// non-zero points of img. Returns the number of points in img that are non-zero.
rFeatures_EXPORT int findBoundingBox( const cv::Mat_<byte>& img, cv::Rect& boundingBox);

// Creates a mask where each point in provided vector pts is set to 255 in the returned
// image (all other pixels being set to zero).
rFeatures_EXPORT cv::Mat_<byte> pointsToMask( const cv::Size& imgSz, const std::vector<cv::Point>& pts);

// Zeros out those areas of inputImg where the corresponding value of mask == 0.
// Images must be the same size, but inputImg can be any depth or number of channels.
rFeatures_EXPORT void setMasked( cv::Mat inputImg, const cv::Mat_<byte> mask);

// Draws the convex hull of the given vector of points as a filled (by default) polygon on the given image.
// img must have dimensions large enough to contain the points given in the vector.
rFeatures_EXPORT void drawConvexHull( const vector<cv::Point>& points, cv::Mat img, cv::Scalar colour, bool filled=true);

rFeatures_EXPORT void drawFilledPoly( const vector<cv::Point>& points, cv::Mat img, cv::Scalar colour);

rFeatures_EXPORT void drawPoly( const vector<cv::Point>& points, cv::Mat img, cv::Scalar colour);

// Create a histogram of the values of the byte image m. The given vector is cleared and resized to 256 before use.
rFeatures_EXPORT void createHistogram( const cv::Mat_<byte>& m, vector<int>& hist);

// Given a histogram with two expected peaks, find the normal distributions (means and std-devs) that
// best match these peaks by performing k-means clustering (where k=2) on the given histogram.
// This function obviously expects that the peaks can be roughly approximated by normal
// distributions if the output is to make sense!
// The function returns the value between the two Gaussian means where they intersect.
// If the function returns < 0, there is no intersection of the Gaussians between their
// means, implying that the given histogram is not bimodal!
//double calcBimodalGaussians( const vector<int>& hist, double& u1, double& s1, double& u2, double& s2);

rFeatures_EXPORT cv::Mat_<cv::Vec3b> drawHistogram( const vector<int>& hist, const cv::Size& imgSz, bool makeProbDist=false);

// Creates and returns a normalised 2D Gaussian of given size.
// xcentre, ycentre: give centre coordinates of the 2D Gaussian as proportion of the given size.
rFeatures_EXPORT cv::Mat_<float> make2DGaussian( const cv::Size filterSz, double xcentre=0.5, double ycentre=0.5);

// Take a N channelled R*C sized input matrix of any depth and turn into a single
// channelled matrix with N rows with each row R*C in length.
// Optional scaling is provided with params alpha and beta (see OpenCV docs on Mat::convertTo).
rFeatures_EXPORT cv::Mat_<float> toRowVectors( const cv::Mat& img, double alpha=1, double beta=0);

// Goes one further than toRowVectors() and turns an N channelled R*C sized input matrix into a single row vector N*R*C in length.
rFeatures_EXPORT cv::Mat_<float> toRowVector( const cv::Mat& img, double alpha=1, double beta=0);

// Set the given pixel (not accounting for image channels) to be scaled to its sqrt
// multiplied by sqrt(MAX_VALUE).
rFeatures_EXPORT void sqrtGammaCorrect( cv::Mat &img, const int MAX_VALUE, int row, int col);

// Gamma correct the given CV_8U image, returning a new image.
rFeatures_EXPORT cv::Mat sqrtGammaCorrect( const cv::Mat& img);

// Gets the bands of the CIE Lab space for the given image.
// Lightness (channel 0) values are in range [0,100].
// Colour a (channel 1) values are in the range [-127,127].
// Colour b (channel 2) values are in the range [-127,127].
rFeatures_EXPORT vector<cv::Mat_<float> > getCIELabChannels( const cv::Mat_<cv::Vec3b>& image);

// In-place conversion to CIE-Lab. All values in all channels in range [0,255].
rFeatures_EXPORT void convertToCIELab( cv::Mat_<cv::Vec3b>& img);

// Gets the CIE Lab lightness channel only with values scaled from 0 to scale.
// Default returned image type is CV_32F
rFeatures_EXPORT cv::Mat getLightness( const cv::Mat_<cv::Vec3b>& image, double scale=1, int imgType=CV_32F);

// Uses DepthSegmenter to create a depth mask from the most common depth values in dimg.
rFeatures_EXPORT cv::Mat_<byte> createDepthMask( const cv::Mat_<float> dimg);

// Type-casted retrieval of a value from a pointer into a matrix.
// Takes the following matrix depths:
// CV_8U, CV_8S, CV_16U, CV_16S, CV_32S, CV_32F, CV_64F
rFeatures_EXPORT double pval( int matrixDepth, const byte* valuePointer);

// Get the horizontal gradient at the given pixel. Boundary pixels take as the gradient
// the difference between the specified pixel and the adjacent inward pixel.
// This function only accepts images having a vector type (cv::Vec) as their element e.g.
// to call for a CV_64FC1 image, the function should be called as
// calcHorizontalGrad<cv::Vec<double,1> >( ...)
template <typename T>
double calcHorizontalGrad( const cv::Mat_<T> &image, int row, int col, int channel);

rFeatures_EXPORT double calcHorizontalGrad( const cv::Mat& img, int row, int col, int channel);

template <typename T>
double calcHorizontalGrad2( const cv::Mat_<T> &image, int row, int col, int channel);

// Get the vertical gradient at the given pixel. Boundary pixels take as the gradient
// the difference between the specified pixel and the adjacent inward pixel.
// See calcHorizontalGrad for details on required type (must be cv::Vec).
template <typename T>
double calcVerticalGrad( const cv::Mat_<T> &image, int row, int col, int channel);

rFeatures_EXPORT double calcVerticalGrad( const cv::Mat& img, int row, int col, int channel);

template <typename T>
double calcVerticalGrad2( const cv::Mat_<T> &image, int row, int col, int channel);

// Display a cv::Rect to an outstream.
template <typename T>
ostream &operator<<( ostream&, const cv::Rect_<T>&);

template <typename T>
istream &operator>>( istream&, cv::Rect_<T>&);

rFeatures_EXPORT ostream &operator<<( ostream&, const cv::Rect&);
rFeatures_EXPORT istream &operator>>( istream&, cv::Rect&);

// Display a cv::Size to an outstream.
template <typename T>
ostream &operator<<( ostream&, const cv::Size_<T>&);

template <typename T>
istream &operator>>( istream&, cv::Size_<T>&);

rFeatures_EXPORT ostream &operator<<( ostream&, const cv::Size&);
rFeatures_EXPORT istream &operator>>( istream&, cv::Size&);

// Writes the given descriptor out the provided stream as single precision
// values separated by spaces on a single line. Does not write newline character.
// Used to write multiple descriptors (one per line) to file.
// Returns the number of values written == descriptor.total()
rFeatures_EXPORT int writeDescriptor( ostream& os, const cv::Mat& descriptor);

// Read descriptors (written using writeDescriptor above) into a returned matrix.
// Descriptors will be set as either column vectors (asCols=true : the default)
// with the number of rows being equal to the length of the feature vectors and
// the number of columns being equal to the number of descriptors read in,
// or as row vectors with the number of columns being equal to the length of
// the feature vectors and the number of rows equalling the number of descriptors read in.
rFeatures_EXPORT cv::Mat_<float> readDescriptors( const string fname, bool asCols=true) throw (DescriptorLengthException);

// Read in a single descriptor as a column or row vector.
rFeatures_EXPORT cv::Mat_<float> readDescriptor( istream& is, bool asCol=true);

// Combine f1 and f2 of different number of columns(rows) but same number of rows(columns)
// into a single row vector of length f1.total() + f2.total()
rFeatures_EXPORT cv::Mat_<float> combine( const cv::Mat_<float>& f1, const cv::Mat_<float>& f2);

// Write a cv::Mat out to the provided stream in binary format.
// Type of matrix does NOT need to be known!
rFeatures_EXPORT ostream &writeBinary( ostream &os, const cv::Mat &m);

rFeatures_EXPORT bool saveBinaryImage( const string fname, const cv::Mat& m);

// Read in a matrix from a stream of binary data. Type does NOT have
// to be known before calling!
rFeatures_EXPORT istream &readBinary( istream &is, cv::Mat &m);

rFeatures_EXPORT bool loadBinaryImage( const string fname, cv::Mat& m);

// Returns depth of image as a string e.g. CV_64F
rFeatures_EXPORT string imgDepthToString( const cv::Mat&);
rFeatures_EXPORT string imgDepthToString( int depth);

// Returns type of image as string (i.e. depth and number of channels) e.g. CV_8UC3
rFeatures_EXPORT string imgTypeToString( const cv::Mat&);

// Load an image from the provided absolute filename into the provided cv::Mat header.
// Set bw to true if image should be loaded in single channel grey scale.
rFeatures_EXPORT bool loadImage( const string &fname, cv::Mat &img, bool bw=false);

// Save the given image to provided filename.
rFeatures_EXPORT bool saveImage( const string &fname, const cv::Mat &img);

// Shows the provided image in its own main window. Set wait to true to
// cause this function to block until user presses a key.
rFeatures_EXPORT void showImage( const cv::Mat &img, const string &title, bool wait=false);

// Close the window displaying the image with the given title.
rFeatures_EXPORT void closeImage( const string& title);

// Display provided image dimensions and number of channels to given output stream.
rFeatures_EXPORT ostream &print( ostream &os, const cv::Mat &img);

// Takes a single or triple channel image and converts it to a CV_8U image. If any channels
// have values outside the displayable range of [0,255], they are first scaled to be within
// this range. The maximum variance out of all channels is used to scale the values over all
// channels (for triple channel images) so channel value scalings are proportionate.
// If forceScale is true, the image will have its values contrast scaled within the
// displayable range of [0,255] even if its values are currently within this range
// (this will result in a contrast stretching of the image values).
rFeatures_EXPORT cv::Mat convertForDisplay( const cv::Mat &img, bool forceScale=false);

// Convert a single channel scalar image (e.g. of type float, double, int etc)
// to a triple channel byte grey scale image ready for display (CV_8UC3).
// Parameter invert does value inversion before mapping to the returned matrix.
rFeatures_EXPORT cv::Mat_<cv::Vec3b> convertFromSingleChannel( const cv::Mat&, bool invert=false);

// Contrast stretch and translate the values of the provided image to lie
// within the provided range [minVal, maxVal].
rFeatures_EXPORT cv::Mat rescale( const cv::Mat img, double minVal=0, double maxVal=255);

// Converts single channel m (any depth) to contrast stretched byte image with
// values in the range 0 to 255. Values to stretch are taken from the given mask.
// If the mask is left empty, the min and max values from the whole image are
// used to contrast stretch.
rFeatures_EXPORT cv::Mat_<byte> contrastStretch( const cv::Mat& m, const cv::Mat_<byte> mask=cv::Mat());

rFeatures_EXPORT cv::Mat_<float> truncateAndScale( const cv::Mat_<float> img, float trunc, float scale=1.0);

// Straight forward function for converting a range map to a contrast scaled CV_8UC3
// for some OpenCV functions.
rFeatures_EXPORT cv::Mat_<cv::Vec3b> makeCV_8UC3( const cv::Mat_<float> rmap);

// Scale image to given size and return
rFeatures_EXPORT cv::Mat scale( const cv::Mat img, const cv::Size& newSz);

// Flattens all N channels of the provided image into a single channel
// by summing each channel/N.
rFeatures_EXPORT cv::Mat flatten( const cv::Mat &img);

// Do cross-convolution of krn with img and return a response map.
rFeatures_EXPORT cv::Mat convolve( const cv::Mat &img, const cv::Mat &krn);

// Shows each image plane of an image one by one.
// Each image is contrast scaled within the range [0,255].
rFeatures_EXPORT void showScaledPlanes( const cv::Mat &img, const string &winNamePrefix);

// Draw the provided rectangles to the provided image with provided
// line thickness (in pixels) and colour (BGR).
rFeatures_EXPORT void drawBoxes( cv::Mat &img, const vector<cv::Rect> &boxes, int thick=1,
                                const cv::Scalar col=cv::Scalar(255,255,255));

// Draws the rotated rectangle to the image, clipping the lines as needed so that the
// rectangle is not drawn outside of the image canvas.
rFeatures_EXPORT void drawRotatedRect( cv::Mat& img, const cv::RotatedRect&, int thick=1,
                                const cv::Scalar col=cv::Scalar(255,255,255));

// Find the x's where two normal distributions (with given std devs and means) cross.
rFeatures_EXPORT void calcNormalsCrossings( double s1, double s2, double u1, double u2, double &x1, double &x2);

// Calculate the value of the normal distribution (with sigma s and mean u) at x.
rFeatures_EXPORT double calcNormal( double s, double u, double x);

// Round v to the closest multiple of m
rFeatures_EXPORT int roundMult( double v, int m);

// Calc and return the projection of p along base.
rFeatures_EXPORT cv::Vec3f project( const cv::Vec3f& p, const cv::Vec3f& base);

// Calculate the sum of the square differences of the provided list of values with the given mean.
// Divide through the returned value with the number of samples (or the number of samples -1) to
// get the variance. calcStdDev and calcStdDevBiased (below) use this function.
rFeatures_EXPORT double calcSumSqDiffs( const vector<double>& vals, double mean);

// Calculate and return std deviation (using n-bias to account for sample bias)
rFeatures_EXPORT double calcStdDev( const vector<double> &vals, double mean, int bias=1);

// Flip every instance of dt about the vertical axis
rFeatures_EXPORT void vertFlipReplace( vector<cv::Mat> &v);

// Create a random colour (BGR) object with given minimum channel intensities
// (set in provided in/out cv::Scalar param).
rFeatures_EXPORT void createRandomColour( cv::Scalar&, rlib::Random& rnd);

// Change whatever fv is into the provided type as a single channel, single row vector
// so that for returned image m, m.cols = m.total(). Parameter alpha allows for element
// scaling on conversion.
rFeatures_EXPORT cv::Mat modifyDescriptor( const cv::Mat &fv, int type=CV_32F, float alpha=1);

// Apply yaw, pitch and roll (in that order) to the given parameter vector and return the result.
// All angles should be given in degrees. Yaw is about Z, pitch is about X, roll is about Y.
rFeatures_EXPORT cv::Vec3d applyYawPitchRoll( const cv::Vec3d& initVec, double yaw, double pitch, double roll);

// Apply roll, pitch and yaw (in that order) to the given parameter vector and return the result.
// All angles should be given in degrees. Yaw is about Z, pitch is about X, roll is about Y.
rFeatures_EXPORT cv::Vec3d applyRollPitchYaw( const cv::Vec3d& initVec, double yaw, double pitch, double roll);

// Apply pitch, roll and yaw (in that order) to the given parameter vector and return the result.
// All angles should be given in degrees. Yaw is about Z, pitch is about X, roll is about Y.
rFeatures_EXPORT cv::Vec3d applyPitchRollYaw( const cv::Vec3d& initVec, double yaw, double pitch, double roll);

// Apply roll, yaw and pitch (in that order) to the given parameter vector and return the result.
// All angles should be given in degrees. Yaw is about Z, pitch is about X, roll is about Y.
rFeatures_EXPORT cv::Vec3d applyRollYawPitch( const cv::Vec3d& initVec, double yaw, double pitch, double roll);

// Apply pitch, yaw and roll (in that order) to the given parameter vector and return the result.
// All angles should be given in degrees. Yaw is about Z, pitch is about X, roll is about Y.
rFeatures_EXPORT cv::Vec3d applyPitchYawRoll( const cv::Vec3d& initVec, double yaw, double pitch, double roll);

// Apply yaw, roll and pitch (in that order) to the given parameter vector and return the result.
// All angles should be given in degrees. Yaw is about Z, pitch is about X, roll is about Y.
rFeatures_EXPORT cv::Vec3d applyYawRollPitch( const cv::Vec3d& initVec, double yaw, double pitch, double roll);

#include "template/FeatureUtils_template.h"

}   // end namespace

#endif
