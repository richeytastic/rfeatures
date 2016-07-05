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
