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
