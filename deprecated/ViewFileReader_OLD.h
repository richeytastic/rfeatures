#pragma once
#ifndef RFEATURES_VIEW_FILE_READER
#define RFEATURES_VIEW_FILE_READER

#include "ViewReader.h"
using RFeatures::ViewReader;


namespace RFeatures
{

class ViewFileReader : public ViewReader
{
public:
    ViewFileReader();
    virtual ~ViewFileReader(){}

    static View::Ptr load( const char *vfile);

protected:
    virtual void read( istream&);
};  // end class

}   // end namespace

#endif
