#include "ViewSampleExtractor.h"
using RFeatures::ViewSampleExtractor;


ViewSampleExtractor::Ptr ViewSampleExtractor::create( const View::Ptr v)
{
    return Ptr( new ViewSampleExtractor( v));
}   // end create


ViewSampleExtractor::ViewSampleExtractor( const View::Ptr v)
    : view_(v), modelName_("N/A"), partName_("N/A"), aspectInfo_("N/A")
{
}   // end ctor



void ViewSampleExtractor::setModelName( const string& nm)
{
    modelName_ = nm;
}   // end setModelName


void ViewSampleExtractor::setPartName( const string& nm)
{
    partName_ = nm;
}   // end setPartName


void ViewSampleExtractor::setAspectInfo( const string& nm)
{
    aspectInfo_ = nm;
}   // end setAspectInfo



ViewExtract::Ptr ViewSampleExtractor::extract( cv::Rect& rct) const
{
    const PointCloud::Ptr vpc = view_->points;

    // Ensure rectangle falls within organised point cloud dimensions
    const cv::Rect pcRect(0,0, vpc->cols(), vpc->rows());
    rct &= pcRect;  // Set intersection

    ViewExtract::Ptr ve( new ViewExtract( vpc, rct));
    ve->setModelName( modelName_);
    ve->setPartName( partName_);
    ve->setAspectInfo( aspectInfo_);
    ve->setPosVec( view_->posVec);
    ve->setDirVec( view_->focalVec);
    ve->setUpVec( view_->upVec);
    return ve;
}   // end extract



