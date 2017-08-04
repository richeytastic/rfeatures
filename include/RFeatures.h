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

#ifndef RFEATURES_RFEATURES_H
#define RFEATURES_RFEATURES_H

#include "rFeatures_Export.h"
#include <cmath>
#include <vector>
#include <string>
#include <exception>
#include <stdexcept>
#include <cassert>
#include <cstdlib>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <opencv2/opencv.hpp>

typedef std::vector<cv::KeyPoint> Keypoints;
typedef std::vector<cv::Vec6f> Lines3d;  // 3D lines (x1,y1,z1,x2,y2,z2)
typedef std::vector<cv::Vec4i> Lines;
typedef std::vector<cv::Vec3f> Circles;

typedef unsigned int uint;
typedef unsigned char byte;

// Disable warnings about standard template library specialisations not being exported in the DLL interface
#ifdef _WIN32
#pragma warning( disable : 4251)
#pragma warning( disable : 4275)
#endif

/*
// DLL export/imports
#ifdef _WIN32
class rFeatures_EXPORT cv::Mat;
class rFeatures_EXPORT cv::KeyPoint;
//template class rFeatures_EXPORT cv::Vec<int, 4>;
//template class rFeatures_EXPORT cv::Vec<float, 6>;
//template class rFeatures_EXPORT cv::Vec<float, 3>;

class rFeatures_EXPORT std::exception;

// std::string
template struct rFeatures_EXPORT std::_Simple_types<char>;
struct rFeatures_EXPORT std::_Container_base12;
template class rFeatures_EXPORT std::_String_val<std::_Simple_types<char> >;
template class rFeatures_EXPORT std::allocator<char>;
template struct rFeatures_EXPORT std::_Wrap_alloc<std::allocator<char> >;
template class rFeatures_EXPORT std::_Compressed_pair<std::_Wrap_alloc<std::allocator<char> >, std::_String_val<std::_Simple_types<char> >, true>;
template struct rFeatures_EXPORT std::char_traits<char>;
template class rFeatures_EXPORT std::basic_string<char, std::char_traits<char>, std::allocator<char> >;

// std::vector types
#define DLLSPEC_STD_VECTOR_TYPE( _dllmacro, _ctype) \
    struct _dllmacro std::_Container_base12; \
    template struct _dllmacro std::_Simple_types<_ctype>; \
    template class _dllmacro std::_Vector_val<std::_Simple_types<_ctype> >; \
    template class _dllmacro std::allocator<_ctype>; \
    template struct _dllmacro std::_Wrap_alloc<std::allocator<_ctype> >; \
    template class _dllmacro std::_Compressed_pair<std::_Wrap_alloc<std::allocator<_ctype> >, std::_Vector_val<std::_Simple_types<_ctype> >, true>; \
    template class _dllmacro std::vector<_ctype>;

DLLSPEC_STD_VECTOR_TYPE( rFeatures_EXPORT, float)
DLLSPEC_STD_VECTOR_TYPE( rFeatures_EXPORT, double)
DLLSPEC_STD_VECTOR_TYPE( rFeatures_EXPORT, int)
DLLSPEC_STD_VECTOR_TYPE( rFeatures_EXPORT, cv::KeyPoint)
DLLSPEC_STD_VECTOR_TYPE( rFeatures_EXPORT, cv::Vec4i)
DLLSPEC_STD_VECTOR_TYPE( rFeatures_EXPORT, cv::Vec6f)
DLLSPEC_STD_VECTOR_TYPE( rFeatures_EXPORT, cv::Vec3f)
#endif
*/
#endif
