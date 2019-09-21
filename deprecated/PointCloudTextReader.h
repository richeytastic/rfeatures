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
#ifndef RFEATURES_POINT_CLOUD_TEXT_READER_H
#define RFEATURES_POINT_CLOUD_TEXT_READER_H

#include "PointCloudReader.h"
#include "rFeatures_Export.h"
using RFeatures::PointCloudReader;


namespace RFeatures
{

class rFeatures_EXPORT PointCloudTextReader : public PointCloudReader
{
public:
    PointCloudTextReader();
    virtual ~PointCloudTextReader(){}

protected:
    virtual void read( istream&);
};  // end class

}   // end namespace

#endif
