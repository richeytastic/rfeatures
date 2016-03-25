#pragma once
#ifndef RFEATURES_IMAGE_TYPE_ENUM_H
#define RFEATURES_IMAGE_TYPE_ENUM_H

#define MAX_RANGE_M 100


namespace RFeatures
{

enum rFeatures_EXPORT ImageType
{
    BGR,    // CV_8UC3
    // The following three image types require CV_8UC3 for conversion in createImageType
    Grey, // Single channel grey value (CV_8UC1) (i.e. flattened BGR)
    Light, // Single channel grey value (CV_32FC1) with values in [0,1]
    CIELab,    // 3 channel CIE Lab (CV_32FC3)

    // The following two image types require CV_32FC1 for conversion in createImageType
    Depth,  // Single channel depth map with values in [0,1] (CV_32FC1)
    EDT,    // Euclidean distance map on range with values in [0,1]
};  // end enum

}   // end namespace

#endif


