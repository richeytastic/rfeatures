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

#ifndef RFEATURES_OBJ_MODEL_WEIGHTS_H
#define RFEATURES_OBJ_MODEL_WEIGHTS_H

/**
 * Maintains an updating set of weights in proportion to the variance between pairs of
 * corresponding vertices in two vertex models where the target model (constructor) is
 * presumed to describe the means of the vertices.
 * The two models must both have the same number of vertices and have no gaps in their
 * vertex IDs (i.e. both must return ObjModel::hasSequentialVertexIds() == true).
 **/

#include "rFeatures_Export.h"
#include <ObjModel.h>

namespace RFeatures {

class InlierWeightCalculator
{
public:
    InlierWeightCalculator( const ObjModel&,    // Model of vertex means (polygon info is ignored).
                            double kappa=2.0,   // Standard deviation cutoff.
                            double C=0.5);      // Distribution width (decrease to increase weights of less well fitting vertices)

    // Calculate and return the updated inlier weights between the target (constructor) model and the model argument.
    // Only vertex IDs that correspond to those on the target model are looked at and so the length of
    // the returned weight set is always the same as the number of vertices in the target model.
    // If the provided model is missing some corresponding vertex IDs, or the provided vertex ID mask is
    // not null and contains entries, then the weights for those vertices are set to zero.
    const cv::Mat_<float>& updateWeights( const ObjModel&, const IntSet* mask=nullptr);

    // Return last calculated weights (all 1's before first call to updateWeights())
    const cv::Mat_<float>& weights() const { return _vw;}

    // Callers can at anytime modify the weights, but this is most appropriate straight after construction.
    // Callers should NOT ADD OR REMOVE entries to this map!
    cv::Mat_<float>& weights() { return _vw;}

    // Return the last calculated squared vertex displacements (all -1 initially).
    // After a call to updateWeights(), if no displacement was calculated for a vertex, it's value is -1.
    const cv::Mat_<float>& squaredDisplacements() const { return _vsd;}

private:
    const ObjModel& _model;
    const double _C;
    const double _lambda;
    cv::Mat_<float> _vw;     // Vertex weights
    cv::Mat_<float> _vsd;    // Vertex displacements
};  // end class

}   // end namespace

#endif
