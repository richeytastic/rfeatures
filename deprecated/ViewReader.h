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
 * Abstract parent class for View reader types.
 *
 * Richard Palmer
 * September 2012
 */

#pragma once
#ifndef RFEATURES_VIEW_READER
#define RFEATURES_VIEW_READER

#include <stdexcept>
#include <iostream>
using std::istream;
#include "PointDataReader.h"
using RFeatures::PointDataReader;
#include "View.h"
using RFeatures::View;
//#include "ImageGradientsBuilder.h"
//using RFeatures::ImageGradientsBuilder;
//#include "RangeGradientsBuilder.h"
//using RFeatures::RangeGradientsBuilder;
//#include "RangeBuilder.h"
//using RFeatures::RangeBuilder;
//#include "IntegralImage.h"
//using RFeatures::IntegralImage;


namespace RFeatures
{

class ViewReader : public PointDataReader
{
public:
    struct Exception : public std::runtime_error
    { Exception( const string& err) : std::runtime_error( err) {} }; // end class

    virtual ~ViewReader(){}

    View::Ptr getView() const { return m_view;}

protected:
    virtual void read( istream &is) = 0;    // Implemented in device specific child classes

    virtual void getSize( istream &is, size_t &width, size_t &height);

    virtual void getPoint( istream &is, size_t row, size_t col,
            double &x, double &y, double &z, double &rng, byte &r, byte &g, byte &b);

    //virtual void finishedRead();

    void setPosition( const cv::Vec3d&);
    void setFocalVector( const cv::Vec3d&);
    void setUpVector( const cv::Vec3d&);

    //void setImageGradientsParams( int nbins, bool dirDep, bool spatialSmooth, bool sqrtGammaCorrect);
    //void setRangeGradientsParams( int nbins, bool dirDep, bool spatialSmooth);
    //void setPointCloud( const PointCloud::Ptr&);

    ViewReader(); // No non-derived class construction

private:
    View::Ptr m_view;
    /*
    ImageGradientsBuilder::Ptr m_imgGradsBuilder;
    RangeGradientsBuilder::Ptr m_rngGradsBuilder;
    RangeBuilder::Ptr m_rngBuilder;
    IntegralImage<int>::Ptr m_rngCntImage;  // Counts range values > 0 only for DepthFinder
    */
};  // end class ViewReader


}   // end namespace

#endif
