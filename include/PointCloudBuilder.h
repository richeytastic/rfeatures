/**
 * Build a structured point cloud.
 * 
 * Richard Palmer
 * September 2012
 */

#pragma once
#ifndef RFEATURES_POINT_CLOUD_BUILDER
#define RFEATURES_POINT_CLOUD_BUILDER

#include <opencv2/opencv.hpp>
#include "PointDataBuilder.h"
using RFeatures::PointDataBuilder;
#include "PointCloud.h"
using RFeatures::PointCloud;
#include "rFeatures_Export.h"



namespace RFeatures
{

class rFeatures_EXPORT PointCloudBuilder : public PointDataBuilder
{
public:
    typedef boost::shared_ptr<PointCloudBuilder> Ptr;

    static PointCloudBuilder::Ptr create();

    virtual void reset( int width, int height);
    virtual void setPointPos( int row, int col, double x, double y, double z);
    virtual void setPointCol( int row, int col, byte r, byte g, byte b);

    inline PointCloud::Ptr getPointCloud() const { return m_pc;}

private:
    PointCloud::Ptr m_pc;
    
    int m_row1, m_row2;
    int m_col1, m_col2;
    double m_x, m_y, m_z;
    byte m_r, m_g, m_b;
    byte m_dataCount;
    void addPoint();

    PointCloudBuilder();
};  // end class

}   // end namespace

#endif
