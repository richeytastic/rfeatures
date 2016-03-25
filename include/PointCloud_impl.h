#pragma once
#ifndef RFEATURES_POINT_CLOUD_IMPL_H
#define RFEATURES_POINT_CLOUD_IMPL_H

#include "PointCloud.h"
using RFeatures::PointCloud;
using RFeatures::PointXYZRGB;
using RFeatures::PointCloudException;

#include <stdint.h>
#include <vector>
using std::vector;

//#include <boost/archive/binary_iarchive.hpp>
//#include <boost/archive/binary_oarchive.hpp>
//#include <boost/serialization/split_member.hpp>


namespace RFeatures
{

class PointCloud_impl : public PointCloud
{
public:
    virtual size_t getWidth() const;
    virtual size_t getHeight() const;

    virtual bool isOrganised() const;

    virtual void resize( size_t rows, size_t cols);
    virtual size_t resize( size_t rows);

    virtual size_t size() const;

    virtual void add( double x, double y, double z, byte r, byte g, byte b) throw (PointCloudException);
    virtual void set( size_t row, size_t col, double x, double y, double z, byte r, byte g, byte b) throw (PointCloudException);

    virtual const PointXYZRGB& at( size_t row, size_t col) const throw (PointCloudException);
    virtual const PointXYZRGB& at( size_t idx) const throw (PointCloudException);
    virtual PointXYZRGB& at( size_t row, size_t col) throw (PointCloudException);
    virtual PointXYZRGB& at( size_t idx) throw (PointCloudException);

    virtual void from( size_t row, size_t col, float &x, float &y, float &z, byte &r, byte &g, byte &b) const throw (PointCloudException);
    virtual void from( size_t row, size_t col, double &x, double &y, double &z, byte &r, byte &g, byte &b) const throw (PointCloudException);

    virtual void from( size_t idx, float &x, float &y, float &z, byte &r, byte &g, byte &b) const throw (PointCloudException);
    virtual void from( size_t idx, double &x, double &y, double &z, byte &r, byte &g, byte &b) const throw (PointCloudException);

    virtual void setColour( size_t row, size_t col, byte r, byte g, byte b) throw (PointCloudException);

    //virtual const vector<PointXYZRGB>& getRaw() const;

private:
    bool _isOrganised;
    size_t _width;
    size_t _height;

    vector<PointXYZRGB> _pdata; // The point data itself

    void init( size_t width, size_t height);

    PointCloud_impl();
    PointCloud_impl( size_t width, size_t height);
    PointCloud_impl( const PointCloud_impl &);
    void operator=( const PointCloud_impl &);

    void checkIndices( const string &methodName, size_t rows, size_t cols) const throw (PointCloudException);

    friend PointCloud::Ptr PointCloud::create();
    friend PointCloud::Ptr PointCloud::create( size_t width, size_t height);

    friend ostream& operator<<( ostream&, const PointCloud::Ptr&);
    friend istream& operator>>( istream&, PointCloud::Ptr&);

    /*
    friend class boost::serialization::access;
    template<class Archive>
    void save( Archive &ar, const unsigned int) const
    {
        ar << _pcloud->width;
        ar << _pcloud->height;

        for ( size_t i = 0; i < _pcloud->size(); ++i)
        {
            ar << _pcloud->points[i].x;
            ar << _pcloud->points[i].y;
            ar << _pcloud->points[i].z;
            ar << _pcloud->points[i].rgb;
        }   // end for
    }   // end save

    template<class Archive>
    void load( Archive &ar, const unsigned int)
    {
        size_t width, height;
        ar >> width >> height;
        init( width, height);

        for ( size_t i = 0; i < _pcloud->size(); ++i)
        {
            ar >> _pcloud->points[i].x;
            ar >> _pcloud->points[i].y;
            ar >> _pcloud->points[i].z;
            ar >> _pcloud->points[i].rgb;
        }   // end for
    }   // end load

    BOOST_SERIALIZATION_SPLIT_MEMBER()
    */
}; // end class

}  // end namespace


#endif
