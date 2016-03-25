/**
 * Abstract parent class for View writer types.
 *
 * Richard Palmer
 * September 2012
 */

#pragma once
#ifndef RFEATURES_VIEW_WRITER_H
#define RFEATURES_VIEW_WRITER_H

#include "PointDataWriter.h"
using RFeatures::PointDataWriter;
#include "View.h"
using RFeatures::View;


namespace RFeatures
{

class ViewWriter : public PointDataWriter
{
public:
    virtual ~ViewWriter();

protected:
    virtual void write( ostream &os) const = 0;   // Implemented in device specific child classes

    explicit ViewWriter( const View::Ptr&); // No non-derived class construction
    const View::Ptr m_view;   // Object being written out
};  // end class


}   // end namespace

#endif
