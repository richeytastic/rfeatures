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
