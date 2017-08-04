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

#include "FeatureExceptions.h"
using RFeatures::Exception;


Exception::Exception( const string &err) : m_err(err){}
Exception::~Exception() throw(){}
const char* Exception::what() const throw(){ return m_err.c_str();}
string Exception::error() const throw(){ return m_err;}
string Exception::errStr() const throw(){ return m_err;}


RFeatures::FeatureException::FeatureException( const string &es) : Exception( es) {}

RFeatures::FeatureSizeException::FeatureSizeException( const string &es) : Exception( es) {}

RFeatures::ScaledFeatureException::ScaledFeatureException( const string &es) : Exception( es) {}

RFeatures::ImageTypeException::ImageTypeException( const string &es) : Exception( es) {}

RFeatures::ImageOutOfBoundsException::ImageOutOfBoundsException( const string &es) : Exception( es) {}

RFeatures::ImageSizeException::ImageSizeException( const string &es) : Exception( es) {}

RFeatures::DescriptorLengthException::DescriptorLengthException( const string &es) : Exception( es) {}

RFeatures::VectorLengthException::VectorLengthException( const string &es) : Exception( es) {}

RFeatures::ExtractorTypeException::ExtractorTypeException( const string& es) : Exception( es) {}
