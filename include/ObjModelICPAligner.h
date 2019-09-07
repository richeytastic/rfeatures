/************************************************************************
 * Copyright (C) 2019 Richard Palmer
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

#ifndef RFEATURES_OBJ_MODEL_ICP_ALIGNER_H
#define RFEATURES_OBJ_MODEL_ICP_ALIGNER_H

#include "ObjModelAligner.h"

namespace RFeatures {

class rFeatures_EXPORT ObjModelICPAligner : public ObjModelAligner
{
public:
    explicit ObjModelICPAligner( const ObjModel&);
    ~ObjModelICPAligner() override;

    // Calculate the transform to map the given object to the constructor target object using ICP.
    cv::Matx44d calcTransform( const ObjModel&) const override;

private:
    int _n;     // Number of target model points
    double* _T;  // The model points as x1,y1,z1,x2,y2,z2,...,xN,yN,zN

    ObjModelICPAligner( const ObjModelICPAligner&) = delete;
    void operator=( const ObjModelICPAligner&) = delete;
};  // end class

}   // end namespace

#endif
