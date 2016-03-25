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
