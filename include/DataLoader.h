/**
 * Loads data from example files where each line of the file is of the form:
 * pano_id|view_id|object_class|object_id|X|Y|width|height
 *
 * Richard Palmer
 * September 2013
 */

#pragma once
#ifndef RFEATURES_DATA_LOADER_H
#define RFEATURES_DATA_LOADER_H

#include "FeatureUtils.h"
#include "DataTools.h"
#include "FeatureExtractor.h"
using RFeatures::FeatureExtractor;
#include <ProgressDelegate.h>
using rlib::ProgressDelegate;

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>


namespace RFeatures
{

struct rFeatures_EXPORT Instance
{
    string viewId;  // Identifier
    View::Ptr view; // Contains view->img2d (CV_8UC3) and view->rngImg (CV_32FC1) of actual range values
    vector<FeatureExtractor::Ptr> fxs; // Pre-processed feature extractors
    cv::Rect boundBox;  // Placement of instance in the view images and fxs
};  // end struct


class rFeatures_EXPORT DataLoader
{
public:
    typedef boost::shared_ptr<DataLoader> Ptr;

    // Directory location of panoram files (.pano files) specified in the examples file.
    static Ptr create( const string& panorama_directory, ProgressDelegate* pd=NULL);
    ~DataLoader();

    void setProgressDelegate( ProgressDelegate* pd);    // To get loading progress

    void load( const string& examplesFile); // Non-blocking

    // Call subsequent to calling load. Returns # of instances pushed into parameter.
    // Blocks until asynchronous function load has finished.
    int getInstances( vector<Instance>& instances);

    const string& getPanoDirectory() const;

private:
    RFeatures::PanoramaReader _panoReader;
    ProgressDelegate* _pupdater;
    vector<RFeatures::FeatureRecord> _frecs;
    vector<Instance> _recs;
    boost::thread _thread;  // Thread for loadInstancesNoBlock

    boost::unordered_map<string, View::Ptr> _views; // View ID mapped to view


    bool setImagesFromView( const string&, const string&);

    friend class boost::thread;
    void createInstances( const vector<RFeatures::FeatureRecord>*);

    DataLoader( const string& panorama_directory, ProgressDelegate*);
};  // end class

}   // end namespace

#endif
