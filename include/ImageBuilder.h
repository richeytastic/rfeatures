#pragma once
#ifndef RFEATURES_IMAGE_BUILDER_H
#define RFEATURES_IMAGE_BUILDER_H

#include "rFeatures_Export.h"
#include <opencv2/opencv.hpp>
#include "PointDataBuilder.h"
using RFeatures::PointDataBuilder;


namespace RFeatures
{

class rFeatures_EXPORT ImageBuilder : public PointDataBuilder
{
public:
    typedef boost::shared_ptr<ImageBuilder> Ptr;

    ImageBuilder( int width, int height);
    static ImageBuilder::Ptr create( int width, int height);

    virtual void setPointCol( int row, int col, byte r, byte g, byte b);
    virtual void reset( int width, int height);

    inline cv::Mat getImage() const { return img;}

private:
    cv::Mat_<cv::Vec3b> img;    // Built 2D image
};  // end class


}   // end namespace

#endif
