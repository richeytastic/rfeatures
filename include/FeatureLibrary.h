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

#ifndef RFEATURES_FEATURE_LIBRARY_H
#define RFEATURES_FEATURE_LIBRARY_H

/**
 * Singleton class where all known feature types are registered for use.
 */

#include <string>
using std::string;
#include <istream>
#include <vector>
using std::vector;
#include <boost/unordered_map.hpp>
using boost::unordered_map;
#include <boost/shared_ptr.hpp>

#include "FeatureExceptions.h"
using RFeatures::ExtractorTypeException;
#include "FeatureUtils.h"

#include "FeatureExtractor.h"
typedef RFeatures::FeatureExtractor::Ptr FX;

#include "GallLempitskyFeatureExtractor.h"
#include "DepthDiffExtractor.h"
#include "GradientExtractor.h"
#include "LocalBinaryPatternExtractor.h"
#include "CircleDiffExtractor.h"
#include "FastHOGExtractor.h"
#include "HOGExtractor.h"
#include "ProHOGExtractor.h"
#include "SobelEdgesExtractor.h"
#include "EDTFeatureExtractor.h"


namespace RFeatures
{

class rFeatures_EXPORT FeatureLibrary
{
public:
    typedef boost::shared_ptr<FeatureLibrary> Ptr;

    static Ptr get();   // Get the shared feature library

    // Read in feature extractors from a file containing the specifications.
    static std::vector<FX> readFXs( const string& filename);

    void registerFX( FX); // Register a feature extractor that can be built by this factory class

    // Build a single feature extractor
    FX build( const string& fxspec) const throw (ExtractorTypeException);

    // Build several (store in fxs) and return number constructed.
    size_t build( const vector<string>& fxspecs, vector<FX>& fxs) const throw (ExtractorTypeException);

    // Build from input stream
    size_t build( istream&, vector<FX>& fxs) const throw (ExtractorTypeException);

private:
    unordered_map<string, FX> _fxMap;  // Registered FXs

    FeatureLibrary()
    {
        registerFX( FX( new RFeatures::GallLempitskyFeatureExtractor));
        registerFX( FX( new RFeatures::DepthDiffExtractor));
        registerFX( FX( new RFeatures::GradientExtractor));
        registerFX( FX( new RFeatures::LocalBinaryPatternExtractor));
        registerFX( FX( new RFeatures::CircleDiffExtractor));
        registerFX( FX( new RFeatures::FastHOGExtractor));
        registerFX( FX( new RFeatures::HOGExtractor));
        registerFX( FX( new RFeatures::ProHOGExtractor));
        registerFX( FX( new RFeatures::SobelEdgesExtractor));
        registerFX( FX( new RFeatures::EDTFeatureExtractor));
    }   // end ctor

    static Ptr S_singleton;
};  // end class

}   // end namespace

#endif




