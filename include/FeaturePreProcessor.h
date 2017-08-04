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
 * Pre-processes the view data of all instances to create the FeatureExtractor objects
 * ready for each instance.
 *
 * Richard Palmer
 * 2014
 */

#pragma once
#ifndef RFEATURES_FEATURE_PRE_PROCESSOR_H
#define RFEATURES_FEATURE_PRE_PROCESSOR_H

#include "FeatureExtractor.h"
typedef RFeatures::FeatureExtractor::Ptr FX;
#include "DataLoader.h"
using RFeatures::Instance;
#include <ProgressDelegate.h>
using rlib::ProgressDelegate;
#include <vector>
using std::vector;
#include <string>
using std::string;

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <boost/unordered_map.hpp>
#include <boost/shared_ptr.hpp>

namespace RFeatures
{

class rFeatures_EXPORT FeaturePreProcessor
{
public:
    typedef boost::shared_ptr<FeaturePreProcessor> Ptr;

    // Return value and references instances and pd must be maintained while background processing being undertaken.
    static Ptr preProcessViews( const vector<FX>& fxs, vector<Instance>& instances, ProgressDelegate* pd=NULL);

    virtual ~FeaturePreProcessor();

private:
    const vector<FX> _fxs;
    boost::thread_group _threadGrp;

    // Mappings of view identifiers to the pre-processed images
    boost::unordered_map<string, vector<FX> > _viewFxs;
    boost::mutex _mutex;

    void threadFunc( Instance*, int, ProgressDelegate*);

    explicit FeaturePreProcessor( const vector<FX>& fxs);

    void process( vector<Instance>& instances, ProgressDelegate* pd=NULL); // Non-blocking if pd set, otherwise blocks
};  // end class

}   // end RFeatures

#endif
