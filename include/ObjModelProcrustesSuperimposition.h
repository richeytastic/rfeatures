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

#ifndef RFEATURES_OBJ_MODEL_PROCRUSTES_SUPERIMPOSITION_H
#define RFEATURES_OBJ_MODEL_PROCRUSTES_SUPERIMPOSITION_H

#include "ObjModelAligner.h"
#include "ObjModelWeights.h"

namespace RFeatures {

// Use Procrustes to fit a model to a target. There must be a one-to-one correspondence
// of vertex indices between the target (constructor) model and the model argument to calcTransform.
class rFeatures_EXPORT ObjModelProcrustesSuperimposition : public ObjModelAligner
{
public:
    // Provide target model to superimpose to and per vertex weights.
    // If scaleUp true, calcTransform will return transformation matrices that scale the
    // argument model to match the target model's dimensions.
    ObjModelProcrustesSuperimposition( const ObjModel&, const std::vector<double>& W, bool scaleUp=false);

    // Calculate the transform to map the given object to the constructor target object
    // with corresponding vertex indices and one step Procrustes superimposition.
    cv::Matx44d calcTransform( const ObjModel&) const override;

private:
    const ObjModel &_model; // Target model.
    bool _scaleUp;          // Whether or not to scale transformed models up to the target dimensions.
    cv::Mat_<double> _A;    // Model vertices as column vectors.
    cv::Mat_<double> _W;    // Column vector weights as a single row vector.
    cv::Vec3d _vbar;        // The centroid of A.
    double _s;              // Target's initial scale.

    ObjModelProcrustesSuperimposition( const ObjModelProcrustesSuperimposition&) = delete;
    void operator=( const ObjModelProcrustesSuperimposition&) = delete;
};  // end class

}   // end namespace

#endif
