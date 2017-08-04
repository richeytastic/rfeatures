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

#pragma once
#ifndef RFEATURES_FEATURE_EXCEPTIONS_H
#define RFEATURES_FEATURE_EXCEPTIONS_H

#include "rFeatures_Export.h"
#include <exception>
#include <string>
using std::string;

// Disable warnings about standard template library specialisations not being exported in the DLL interface
#ifdef _WIN32
#pragma warning( disable : 4251)
#pragma warning( disable : 4275)
#endif

namespace RFeatures
{

class rFeatures_EXPORT Exception : public std::exception
{
public:
    Exception( const string &err);
    virtual ~Exception() throw();
    virtual const char* what() const throw();
    virtual string error() const throw();
    virtual string errStr() const throw();
private:
    string m_err;
}; // end class


class rFeatures_EXPORT FeatureException : public Exception
{
public:
    FeatureException( const string &es);
};  // end class


class rFeatures_EXPORT FeatureSizeException : public Exception
{
public:
    FeatureSizeException( const string &es);
};  // end class


class rFeatures_EXPORT ScaledFeatureException : public Exception
{
public:
    ScaledFeatureException( const string &es);
};  // end class


class rFeatures_EXPORT ImageTypeException : public Exception
{
public:
    ImageTypeException( const string &es);
};  // end class


class rFeatures_EXPORT ImageOutOfBoundsException : public Exception
{
public:
    ImageOutOfBoundsException( const string &es);
};  // end class


class rFeatures_EXPORT ImageSizeException : public Exception
{
public:
    ImageSizeException( const string &es);
};  // end class


class rFeatures_EXPORT DescriptorLengthException : public Exception
{
public:
    DescriptorLengthException( const string &es);
};  // end class


class rFeatures_EXPORT VectorLengthException : public Exception
{
public:
    VectorLengthException( const string &es);
};  // end class


class rFeatures_EXPORT ExtractorTypeException : public Exception
{
public:
    ExtractorTypeException( const string& es);
};  // end class

}   // end namespace

#endif
