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
 * Store details about an extract of a view (e.g. for model training)
 * offering IO stream operators.
 *
 * Richard Palmer
 * November 2012
 */

#pragma once
#ifndef RFEATURES_VIEW_EXTRACT_H
#define RFEATURES_VIEW_EXTRACT_H

#include <string>
using std::string;
#include <vector>
using std::vector;
#include <iostream>
#include <fstream>

#include <opencv2/opencv.hpp>

#include <boost/shared_ptr.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>

#include "PointCloud.h"
using RFeatures::PointCloud;


namespace RFeatures
{

class ViewExtract
{
public:
    typedef boost::shared_ptr<ViewExtract> Ptr;

    // Create example from a bounded region of point cloud data.
    ViewExtract( const PointCloud::Ptr, const cv::Rect&);
    ViewExtract();    // Used for operator>>

    // Optional data giving description of the model to which this example belongs,
    // the particular component part of the model (if any) and any information about
    // the aspect (view direction) of the example (e.g. front, left, rear etc).
    void setModelName( const string&);
    void setPartName( const string&);
    void setAspectInfo( const string&);

    inline string getModelName() const { return modelName_;}
    inline string getPartName() const { return partName_;}
    inline string getAspectInfo() const { return aspectInfo_;}

    // Set vectors that allow for knowledge of example pose.
    void setPosVec( const cv::Vec3d&);
    void setDirVec( const cv::Vec3d&);
    void setUpVec( const cv::Vec3d&);

    inline cv::Vec3d getPosVec() const { return posVec_;}
    inline cv::Vec3d getDirVec() const { return dirVec_;}
    inline cv::Vec3d getUpVec() const { return upVec_;}

    // Attempt to calculate the real size of this extract in metres.
    // Takes focal length of camera as parameter (256 for 512 pixel
    // square images and a 90 degree field of view). This is calculated
    // by taking the mean depth of the extract from the 9 pixels at the
    // centre of the extract and calculating based on similar triangles.
    cv::Size2f calcRealSize( double focLen=256) const;

    inline const PointCloud::Ptr getData() const { return data_;}

    void save( const char *vfile) const;    // Save this ViewExtract to file

    // Load a new view extract from file
    static ViewExtract::Ptr load( const char *vfile);

    // Load a bunch of view extracts from a directory and place into the provided
    // vector. All regular files inside the given directory are expected to be
    // view extract files.
    static void loadDir( const char *vdir, vector<ViewExtract::Ptr> &);

private:
    PointCloud::Ptr data_;  // Actual point cloud data of the example (relative to position)
    cv::Rect bbox_;         // Bounding box defining example area of point cloud
    string modelName_;      // Name of the model the example is an instance of [OPTIONAL]
    string partName_;       // What part of the model [OPTIONAL]
    string aspectInfo_;     // Info concerning the orientation of the example [OPTIONAL]
    cv::Vec3d posVec_;      // Absolute position vector of view location
    cv::Vec3d dirVec_;      // Focal vector of the view direction from posVec_
    cv::Vec3d upVec_;       // Up vector (orthogonal to dirVec_)

    friend std::ostream& operator<<( std::ostream&, const ViewExtract&);
    friend std::istream& operator>>( std::istream&, ViewExtract&);
}; // end class


// Write example out to stream in text format
std::ostream& operator<<( std::ostream&, const ViewExtract&);

// Read in example from stream
std::istream& operator>>( std::istream&, ViewExtract&);

}  // end namespace

#endif
