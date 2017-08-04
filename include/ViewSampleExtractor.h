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
 * Extract samples bounded by rectangular regions from views.
 * Richard Palmer
 * 2013
 */

#pragma once
#ifndef RFEATURES_VIEW_SAMPLE_EXTRACTOR_H
#define RFEATURES_VIEW_SAMPLE_EXTRACTOR_H

#include "View.h"
using RFeatures::View;
#include "ViewExtract.h"
using RFeatures::ViewExtract;


namespace RFeatures
{

class ViewSampleExtractor
{
public:
    typedef boost::shared_ptr<ViewSampleExtractor> Ptr;

    static Ptr create( const View::Ptr);
    explicit ViewSampleExtractor( const View::Ptr);

    // ViewExtract objects created by this Extractor will have their
    // model description info set to the values provided to these functions.
    // New values can be set at any time and all subsequent extracts will
    // use the new values. Initial values are set to "N/A".
    void setModelName( const string&);
    void setPartName( const string&);
    void setAspectInfo( const string&);

    // Extract sample delimted by given bounds. If bounds are outside
    // of the point cloud stored by the view, the rectangle is modified
    // for return to indicate the bounds of the view extracted.
    ViewExtract::Ptr extract( cv::Rect&) const;

    inline View::Ptr getView() const { return view_;}

private:
    const View::Ptr view_;
    string modelName_;
    string partName_;
    string aspectInfo_;
};  // end class

}   // end namespace

#endif
