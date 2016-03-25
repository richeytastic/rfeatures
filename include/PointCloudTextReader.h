#pragma once
#ifndef RFEATURES_POINT_CLOUD_TEXT_READER_H
#define RFEATURES_POINT_CLOUD_TEXT_READER_H

#include "PointCloudReader.h"
#include "rFeatures_Export.h"
using RFeatures::PointCloudReader;


namespace RFeatures
{

class rFeatures_EXPORT PointCloudTextReader : public PointCloudReader
{
public:
    PointCloudTextReader();
    virtual ~PointCloudTextReader(){}

protected:
    virtual void read( istream&);
};  // end class

}   // end namespace

#endif
