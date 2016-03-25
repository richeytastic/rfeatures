#pragma once
#ifndef RFEATURES_LAS_READER_H
#define RFEATURES_LAS_READER_H

#include "rFeatures_Export.h"
#include <string>
using std::string;
#include <exception>
#include <opencv2/opencv.hpp>

#include "PointCloud.h"
using RFeatures::PointCloud;

#include <liblas/liblas.hpp>
#include <liblas/reader.hpp>
typedef liblas::Point LASPoint;

namespace RFeatures
{

class rFeatures_EXPORT LASException : public std::exception
{
public:
    LASException( const string &err) : m_err(err){}
    virtual ~LASException() throw(){}
    virtual const char* what() const throw(){ return m_err.c_str();}
    virtual string error() const throw(){ return m_err;}
    virtual string errStr() const throw(){ return m_err;}
private:
    string m_err;
}; // end class


class rFeatures_EXPORT LASReader
{
public:
    LASReader( const string& fname);

    // Other info available after this function called.
    PointCloud::Ptr read() throw (LASException);

    double getMinTime() const { return _minTime;}
    double getMaxTime() const { return _maxTime;}

    cv::Vec3d getOffset() const { return _offset;}
    cv::Vec3d getScale() const { return _scale;}

    double getMaxX() const { return _maxX;}
    double getMinX() const { return _minX;}
    double getMaxY() const { return _maxY;}
    double getMinY() const { return _minY;}
    double getMaxZ() const { return _maxZ;}
    double getMinZ() const { return _minZ;}

private:
    const string _fname;
    double _minTime, _maxTime;
    double _minX, _maxX, _minY, _maxY, _minZ, _maxZ;
    cv::Vec3d _offset, _scale;

    void testPointBounds( const cv::Vec3d&);
    void readPointTime( const LASPoint&);

    void readPointFormat0( const LASPoint&, PointCloud::Ptr);
    void readPointFormat1( const LASPoint&, PointCloud::Ptr);
    void readPointFormat2( const LASPoint&, PointCloud::Ptr);
    void readPointFormat3( const LASPoint&, PointCloud::Ptr);
};  // end class

}   // end namespace

#endif


