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

/**
 * Abstract parent class for PointCloud reader types.
 *
 * Richard Palmer
 * November 2012
 */

#pragma once
#ifndef RFEATURES_POINT_CLOUD_READER
#define RFEATURES_POINT_CLOUD_READER

#include <exception>
#include <string>
using std::string;
#include "PointDataReader.h"
using RFeatures::PointDataReader;
#include "PointCloud.h"
using RFeatures::PointCloud;
#include "rFeatures_Export.h"


namespace RFeatures
{

class rFeatures_EXPORT PointCloudReader : public PointDataReader
{
public:
    typedef boost::shared_ptr<PointCloudReader> Ptr;

    class Exception : public std::exception
    {
    public:
        Exception( const string &err) : m_err(err){}
        virtual ~Exception() throw(){}
        virtual const char* what() const throw(){ return m_err.c_str();}
        virtual string error() const throw(){ return m_err;}
        virtual string errStr() const throw(){ return m_err;}
    private:
        string m_err;
    }; // end class Exception


    virtual ~PointCloudReader(){}

    // Get the constructed point cloud
    inline PointCloud::Ptr getPointCloud() const { return pcloud_;}

protected:
    virtual void read( istream &is) = 0;    // Implemented in device specific child classes

    virtual void getSize( istream &is, size_t &width, size_t &height);

    virtual void getPoint( istream &is, size_t row, size_t col,
            double &x, double &y, double &z, double &rng, byte &r, byte &g, byte &b);

    virtual void finishedRead();


    // Called by child classes while reading in data
    void createPointCloud( size_t rows, size_t cols);
    void setPoint( size_t row, size_t col, double x, double y, double z, byte r, byte g, byte b);

    PointCloudReader(); // No non-derived class construction

private:
    PointCloud::Ptr pcloud_;
};  // end class PointCloudReader


//istream &operator>>( istream&, PointCloudReader::Ptr&);

}   // end namespace

#endif
