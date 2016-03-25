/**
 * Writes Views to disk.
 * Richard Palmer
 * September 2012
 */

#pragma once
#ifndef RFEATURES_VIEW_FILE_WRITER_H
#define RFEATURES_VIEW_FILE_WRITER_H

#include "ViewWriter.h"
using RFeatures::ViewWriter;


namespace RFeatures
{

class ViewFileWriter : public ViewWriter
{
public:
    explicit ViewFileWriter( const View::Ptr&);
    virtual ~ViewFileWriter(){}

protected:
    virtual void write( ostream &os) const;
}; // end class

}  // end namespace

#endif
