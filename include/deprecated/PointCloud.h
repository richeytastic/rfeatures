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

#ifndef RFEATURES_POINT_CLOUD_H
#define RFEATURES_POINT_CLOUD_H

#ifdef _WIN32
#pragma warning( disable : 4290) // Disable warnings about MSVC compiler not implementing exception specifications.
#pragma warning( disable : 4275) // Disable warnings about non dll-interface class ... used as base for dll-interface class.
#pragma warning( disable : 4251) // Disablw warnings about members not having dll interfaces.
#endif

#include <iostream>
using std::ostream;
using std::istream;
#include <exception>
#include <string>
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
#include <boost/shared_ptr.hpp>
#include "rFeatures_Export.h"

typedef unsigned char byte;


namespace RFeatures
{

class rFeatures_EXPORT PointCloudException : public std::exception
{
public:
    PointCloudException( const std::string &err) : m_err(err){}
    virtual ~PointCloudException() throw(){}
    virtual const char* what() const throw(){ return m_err.c_str();}
    virtual std::string error() const throw(){ return m_err;}
    virtual std::string errStr() const throw(){ return m_err;}
private:
    std::string m_err;
}; // end class

typedef struct {union{ float data[4]; struct { float x; float y; float z;};}; union{ struct{ float rgb;}; float data_c[4];};} PointXYZRGB;


class rFeatures_EXPORT PointCloud
{
public:
    typedef boost::shared_ptr<PointCloud> Ptr;

    static const Ptr Null;

    // No initial size (not structured) - can only use add - not set!
    static Ptr create();

    // Create a new organised point cloud and reserve space of the given 2D dimensions.
    // Can only use set - not add!
    static Ptr create( size_t width, size_t height);

    // Pack and unpack points
    static PointXYZRGB &packPoint( PointXYZRGB &pt,
                            double x, double y, double z, byte r, byte g, byte b);
    static void unpackPoint( const PointXYZRGB &pt,
                            float &x, float &y, float &z, byte &r, byte &g, byte &b);
    static void unpackPoint( const PointXYZRGB &pt,
                            double &x, double &y, double &z, byte &r, byte &g, byte &b);


    // For organised point clouds (e.g. range data), width and height
    // give the dimensions of the 2D face of the point cloud volume
    // (i.e. pixel indices from the flat 2D image).
    virtual size_t getWidth() const = 0;
    virtual size_t getHeight() const = 0;
    size_t rows() const { return getHeight();}  // Alias to getHeight()
    size_t cols() const { return getWidth();}   // Alias to getWidth()

    // Returns true iff this represents an organised (i.e. with height > 1) point cloud.
    virtual bool isOrganised() const = 0;

    // Sets point cloud as organised if rows > 1, otherwise sets as unorganised.
    // This method resizes the point cloud to the required dimensions. All existing
    // points outside of these dimensions will be lost! To retain existing points,
    // use resize(rows).
    virtual void resize( size_t rows, size_t cols) = 0;

    // As resize, but retains all existing points. If all points cannot be contained
    // in the specified number of rows with the same number of points per row, an
    // extra row will be added to contain the excess. For example, with a point cloud
    // having original unorganised dimensions of height=1 and width=20, calling resize(3)
    // will give a point cloud of height=4 and width=6 with the final row occupied with
    // only 2 points. This method returns the actual number of rows (height) set.
    virtual size_t resize( size_t rows) = 0;

    // Return the number of points in this point cloud.
    virtual size_t size() const = 0;

    // Append a point - ONLY for unorganised (non-structured) point clouds!
    // Throws exception if point cloud is organised (structured).
    virtual void add( double x, double y, double z, byte r, byte g, byte b) throw (PointCloudException) = 0;

    // Add a point to this organised (structured) point cloud at the given position.
    // Throws exception if point cloud is not organised or coords are invalid.
    virtual void set( size_t row, size_t col,
            double x, double y, double z, byte r, byte g, byte b) throw (PointCloudException) = 0;

    // Retrieve a reference to a point.
    // Works with both organised and unorganised point clouds.
    // Throws exceptions if indices out of bounds.
    virtual const PointXYZRGB& at( size_t row, size_t col) const throw (PointCloudException) = 0;
    virtual PointXYZRGB& at( size_t row, size_t col) throw (PointCloudException) = 0;
    virtual const PointXYZRGB& at( size_t idx) const throw (PointCloudException) = 0;
    virtual PointXYZRGB& at( size_t idx) throw (PointCloudException) = 0;

    // Retrieve values from a point.
    // Works only for organised point clouds.
    // Throws if indices out of bounds or not an organised point cloud.
    virtual void from( size_t row, size_t col,
            float &x, float &y, float &z, byte &r, byte &g, byte &b) const throw (PointCloudException) = 0;
    virtual void from( size_t row, size_t col,
            double &x, double &y, double &z, byte &r, byte &g, byte &b) const throw (PointCloudException) = 0;

    // Retrieve values from a point.
    // Works with both organised and unorganised point clouds.
    // Throws if idx out of bounds.
    virtual void from( size_t idx,
            float &x, float &y, float &z, byte &r, byte &g, byte &b) const throw (PointCloudException) = 0;
    virtual void from( size_t idx,
            double &x, double &y, double &z, byte &r, byte &g, byte &b) const throw (PointCloudException) = 0;

    // Set the colour of the indicated point.
    // Works with both organised and unorganised point clouds.
    // Throws if indices out of bounds.
    virtual void setColour( size_t row, size_t col, byte r, byte g, byte b) throw (PointCloudException) = 0;

    // Get a const shared reference to the internal point cloud vector
    //virtual const vector<PointXYZRGB>& getRaw() const = 0;

protected:
    ~PointCloud(){} // Doesn't need to be virtual since doesn't need to be called
}; // end class

rFeatures_EXPORT ostream& operator<<( ostream&, const PointCloud::Ptr&);
rFeatures_EXPORT istream& operator>>( istream&, PointCloud::Ptr&);


}  // end namespace


#endif
