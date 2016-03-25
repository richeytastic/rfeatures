#pragma once
#ifndef RFEATURES_GALL_LEMPITSKY_FEATURE_H
#define RFEATURES_GALL_LEMPITSKY_FEATURE_H

#include "FeatureOperator.h"
typedef unsigned char byte;
#include <vector>
#include "FeatureExceptions.h"
using RFeatures::ImageTypeException;


namespace RFeatures
{

class rFeatures_EXPORT GallLempitskyFeature : public RFeatures::FeatureOperator
{
public:
    // Takes a CV_8UC3
    explicit GallLempitskyFeature( const cv::Mat image);
    virtual ~GallLempitskyFeature();

protected:
    virtual void getSampleChannels( const cv::Rect&, std::vector<cv::Mat>&) const;

private:
    std::vector<IplImage*> _channels;  // Descriptor feature channels (32)
};  // end class

}   // end namespace

#endif




