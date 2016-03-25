#pragma once
#ifndef RFEATURES_RFEATURES_H
#define RFEATURES_RFEATURES_H

#include <cmath>
#include <vector>
#include <opencv2/opencv.hpp>
#include "rFeatures_Export.h"

typedef std::vector<cv::KeyPoint> Keypoints;
typedef std::vector<cv::Vec4i> Lines;
typedef std::vector<cv::Vec3f> Circles;

typedef unsigned int uint;
typedef unsigned char byte;

#endif
