#ifndef RFEATURES_LINES_CONVERTER_H
#define RFEATURES_LINES_CONVERTER_H

#include "RFeatures.h"

namespace RFeatures
{

class rFeatures_EXPORT InvalidImageException: public std::exception
{
public:
    InvalidImageException( const cv::Mat &img, const std::string &err) : m_err(err), m_img(img) {}
    virtual ~InvalidImageException() throw(){}
    virtual const char* what() const throw(){ return m_err.c_str();}
    virtual std::string error() const throw(){ return m_err;}
    virtual std::string errStr() const throw(){ return m_err;}
    virtual cv::Mat errImg() const throw(){ return m_img;}
private:
    std::string m_err;
    cv::Mat m_img;
}; // end class InvalidImageException


class rFeatures_EXPORT LinesConverter
{
public:
    LinesConverter( const Lines &lns, const cv::Mat &rngData, float minLen, float focLen)
        throw (InvalidImageException);

    // Converts the provided 2D lines into 3D lines using the provided range data
    // (which should be of type CV_32FC1 i.e. single channel floating point).
    // 2D lines having invalid depth at their endpoints are trimmed until both
    // endpoints are valid. 2D lines having invalid depth part way along are split
    // into separate whole lines. The modified 2D lines vector and the  vector of
    // 3D lines returned from get3DLines will therefore only contain lines having
    // valid depth at every point along their length (whether or not the depth at
    // each point should be treated as part of the line is ignored - see fitAndFilter).
    // All lines found to be shorter than minLen are discarded (3D lines will always
    // be at least as long as their 2D projection counterpart).
    // Parameter focLen is needed to undo the perspective projection of points for
    Lines3d get3DLines() const;

    // Fits 3D lines to their depth points and discards outliers into the bargain.
    // The returned lines are in order of best fit (best fit first). Only lines
    // with R^2 values greater than or equal to the provided value are returned.
    // R^2 is always in [0,1]
    Lines3d fitAndFilter( double minRSquared) const;

private:
    cv::Mat rngData;
    Lines lines;
    float minLen;
    float focLen;
}; // end class

}  // end namespace


#endif


