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

namespace RFeatures
{

class rFeatures_EXPORT ObjModelAligner
{
public:
    typedef boost::shared_ptr<ObjModelAligner> Ptr;

    // Set source model for alignment. Model must have a minimum of five vertices.
    static Ptr create( const ObjModel::Ptr);
    explicit ObjModelAligner( const ObjModel::Ptr);
    ~ObjModelAligner();

    // Get the transform matrix required to map the given object to the constructor source object according to ICP.
    cv::Matx44d operator()( const ObjModel::Ptr) const;
    cv::Matx44d calcTransform( const ObjModel::Ptr) const;  // Synonymous

private:
    const ObjModel::Ptr _model;
    double* _T;   // The model points as x1,y1,z1,x2,y2,z2,...,xN,yN,zN

    ObjModelAligner( const ObjModelAligner&);   // NO COPY
    void operator=( const ObjModelAligner&);    // NO COPY
};  // end class

}   // end namespace

#endif
