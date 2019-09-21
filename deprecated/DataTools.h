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

#pragma once
#ifndef RFEATURES_DATA_TOOLS_H
#define RFEATURES_DATA_TOOLS_H

#include <string>
using std::string;
#include <iostream>
using std::ostream;
using std::istream;
#include <fstream>
using std::ofstream;
#include <vector>
using std::vector;
#include <list>
using std::list;
#include <stdexcept>
#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/unordered_set.hpp>
using boost::unordered_set;
#include <boost/unordered_map.hpp>
using boost::unordered_map;
#include <boost/foreach.hpp>
#include <map>

#include <Convert.h>    // RLIB
#include "rFeatures_Export.h"
#include "Panorama.h"
using RFeatures::Panorama;
#include "View.h"
using RFeatures::View;


namespace RFeatures
{

struct rFeatures_EXPORT FeatureRecord
{
    string dataId;
    string viewInfo;
    string className;
    int exampleId;
    cv::Rect boundingBox;
};  // end struct


// Load feature records into provided vector returning number loaded or -1 on error.
int loadFeatureRecords( const std::string& gtfile, std::vector<FeatureRecord>& frecs);


ostream& operator<<( ostream&, const FeatureRecord&);
istream& operator>>( istream&, FeatureRecord&);


class rFeatures_EXPORT FeatureWriter
{
public:
    explicit FeatureWriter( const string &featDir);
    ~FeatureWriter();

    string getGroundTruthDir() const;

    // Sets the object class name for the next lot of features
    void setClass( const string &className);
    string getClass() const { return className_;}

    // Set the data record related to the feature instances
    void setDataId( const string&);
    string getDataId() const { return dataId_;}

    // Set the view (into the data record) related to the feature instances
    void setViewInfo( const string&);
    string getViewInfo() const { return viewInfo_;}

    // Outputs this particular record, returning the ID of the instance.
    int recordInstance( const cv::Rect&);

    // Erase the feature records matching the given rectangle, dataId and viewInfo.
    // Should only be 1 instance! Returns the number of records erased (0 or 1).
    int eraseInstance( const cv::Rect&, const string& dataId, const string& viewInfo);

    // Find records for the currently set class (see setClass above) with given dataId.
    void findRecords( const string &dataId, vector<FeatureRecord>&) const;

    // Returns true if the provided class name already exists in the parent directory
    // (i.e. a directory with the name of this class already exists)
    bool classExists( const string&) const;

    // Returns all existing class names in the parent directory (passed to constructor).
    // All underscores in the directory (class) names are parsed to spaces.
    void getExistingClassNames( vector<string>&) const;

    string getSaveLocation() const; // The current save location (given the set class)

private:
    boost::filesystem::path featDir_;
    string dataId_;
    string viewInfo_;
    string className_;

    // Records ordered by class type (classRecs_) and by data ID (dataRecs_)
    unordered_map<string, list<FeatureRecord*>* > classRecs_;
    unordered_map<string, list<FeatureRecord*>* > dataRecs_;

    int writeData() const;   // Writes all data to disk - returns >= 0 on success denoting # features written
    FeatureRecord* createNewFeatureRecord( const cv::Rect&);
    bool setGroundTruthDir( const string& featDir);
};  // end class



class rFeatures_EXPORT PanoramaReader
{
public:
    explicit PanoramaReader( const string &panoDir);

    View::Ptr readView( const string& panoView); // panoView give an somepanoId-V where V is one of 0,1,2,3
    View::Ptr readViewFace( const string &panoId, const string &face);
    View::Ptr readViewFace( const string &panoId, int face);

    inline const string& getPanoDirectory() const { return panoDir_;}

    static Panorama::Ptr read( const string& filename);
    static View::Ptr getViewFace( const Panorama::Ptr pano, const string &face);
    static View::Ptr getViewFace( const Panorama::Ptr pano, int face);

    static std::string getViewFaceString( int faceId/*0,1,2,3*/);   // Returns FRONT,LEFT,REAR,RIGHT for 0,1,2,3

    // Create panoId-f where f in {0,1,2,3}
    static std::string createViewString( const std::string& panoId, int faceId);
    // Inverse operation of createViewString()
    static void splitViewString( const std::string& viewStr, std::string& panoId, int& faceId);

private:
    const string panoDir_;
    string lastPanoId_;
    Panorama::Ptr pano_;
};  // end class


struct DataRecord
{
    int classId;
    cv::Size2f size;    // Actual size (dimensions in metres)
    View::Ptr view;     // From from which record comes
    FeatureRecord feature;  // Feature record info
};  // end struct


// Reads in all data records from the given example file. If minSz is not NULL, only examples
// with pixel dimensions strictly larger than *minSz in both directions will be read in.
cv::Size2f readDataRecords( const string &panoDir, const string &exFile, vector<DataRecord> &records,
                            float maxDepth=100, const cv::Size *minSz=NULL);

}   // end namespace


#endif
