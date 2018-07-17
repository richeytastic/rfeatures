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

#ifndef RFEATURES_OBJ_MODEL_ALIGNER_H
#define RFEATURES_OBJ_MODEL_ALIGNER_H

#include "ObjModel.h"

namespace RFeatures {

class rFeatures_EXPORT ObjModelAligner
{
public:
    typedef std::shared_ptr<ObjModelAligner> Ptr;
    // Source model for alignment must have a minimum of five vertices.
    static Ptr create( const ObjModel*);
    static Ptr create( ObjModel::Ptr);
    explicit ObjModelAligner( const ObjModel*);
    ~ObjModelAligner();

    // Calculate the transform to map the given object to the constructor source object using ICP.
    cv::Matx44d calcTransform( const ObjModel*) const;

private:
    int _n;     // Number of source model points
    double* _T; // The model points as x1,y1,z1,x2,y2,z2,...,xN,yN,zN

    ObjModelAligner( const ObjModelAligner&) = delete;
    void operator=( const ObjModelAligner&) = delete;
};  // end class

}   // end namespace

#endif
